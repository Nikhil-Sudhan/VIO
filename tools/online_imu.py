"""Strict incremental FIFO decoding and causal clock mapping for live input.

Raw observations must still be preserved by the acquisition process. Nothing in
this module estimates camera/IMU offset or internal measurement/filter delay.
"""
from collections import deque
import struct
import numpy as np


class FifoDecoder:
    def __init__(self):
        self.current = None
        self.previous = None
        self.last_tick = None
        self.wraps = 0

    def emit(self):
        c = self.current
        if c is None or not {'ticks','gyro','accel','count','receipt_ns'} <= c.keys():
            raise ValueError('Incomplete interior FIFO time slot')
        if self.previous:
            if (c['count']-self.previous['count'])%4 != 1 or c['ticks']-self.previous['ticks'] != 192:
                raise ValueError('FIFO counter/timestamp discontinuity')
        if any(abs(v)>=32760 for v in (*c['gyro'],*c['accel'])):
            raise ValueError('IMU near full scale')
        if any(abs(v)*.0175>500 for v in c['gyro']):
            raise ValueError('Gyro outside configured nominal range')
        self.previous = c
        return c

    def push(self,record):
        if record['kind'] != 'fifo': raise ValueError('Expected raw FIFO batch')
        status = bytes.fromhex(record['status_hex'])
        if len(status)!=2 or record['level_words'] != status[0] | ((status[1]&3)<<8):
            raise ValueError('FIFO status/level mismatch')
        if bool(status[1]&0x68) != record['overflow_or_full'] or record['overflow_or_full']:
            raise ValueError('FIFO overflow/full or inconsistent status')
        payload = bytes.fromhex(record['raw_hex'])
        if len(payload)!=record['read_words']*7 or not 0<=record['read_words']<=min(128,record['level_words']):
            raise ValueError('Invalid FIFO payload length')
        output=[]
        for i in range(0,len(payload),7):
            word=payload[i:i+7];tag=word[0]>>3;count=(word[0]>>1)&3
            if word[0].bit_count()%2: raise ValueError('FIFO tag parity error')
            if tag==4:
                if self.current: output.append(self.emit())
                tick=int.from_bytes(word[1:5],'little')
                if self.last_tick is not None and tick<self.last_tick:
                    if self.last_tick-tick<=2**31: raise ValueError('Sensor timestamp reversed')
                    self.wraps+=2**32
                self.last_tick=tick
                if word[5:]!=b'\x00\x55': raise ValueError('Unexpected FIFO batching rates')
                self.current={'ticks':tick+self.wraps,'count':count,'receipt_ns':record['host_after_boot_ns']}
            elif tag in (1,2):
                name='gyro' if tag==1 else 'accel'
                if not self.current or self.current['count']!=count or name in self.current:
                    raise ValueError('FIFO order/duplicate/counter error')
                self.current[name]=struct.unpack('<3h',word[1:])
                self.current['receipt_ns']=record['host_after_boot_ns']
            else: raise ValueError(f'Unexpected FIFO tag {tag}')
        return output

    def finish(self):
        if self.current is None: return [],False
        if 'gyro' not in self.current or 'accel' not in self.current:
            self.current=None;return [],True
        result=self.emit();self.current=None
        return [result],False


class CausalClock:
    """Warm up over >=1.5 s, then fit four recent prior training anchors.

    Retain three seconds for observation sufficiency checks. The shorter fit
    follows measured oscillator-rate changes without relaxing the 1 ms prior
    prediction limit, stale-clock check, or monotonic output requirement.
    """
    def __init__(self):
        self.anchors=deque()
        self.index=0
        self.last_tick=None
        self.last_host=None
        self.wraps=0
        self.model=None
        self.last_emitted=None
        self.last_usable_host=None
        self.validation_residuals=[]

    def anchor(self,r):
        tick=int(r['sensor_ticks_u32']);before=int(r['host_before_boot_ns']);after=int(r['host_after_boot_ns'])
        if not 0<=tick<2**32 or after<=before or (self.last_host is not None and before<=self.last_host):
            raise ValueError('Invalid clock observation')
        if self.last_tick is not None and tick<self.last_tick:
            if self.last_tick-tick<=2**31: raise ValueError('Clock counter reversed/reset')
            self.wraps+=2**32
        self.last_tick=tick;self.last_host=before;tick+=self.wraps
        midpoint=(before+after)/2
        usable=after-before<=2_000_000
        held_out=self.index%5==0;self.index+=1
        if usable:
            self.last_usable_host=midpoint
            if held_out and self.model is not None:
                residual=midpoint-self.predict(tick)
                self.validation_residuals.append(residual)
                if abs(residual)>1_000_000: raise ValueError('Causal clock prediction residual exceeds 1 ms')
            if not held_out:self.anchors.append((tick,midpoint))
        while self.anchors and midpoint-self.anchors[0][1]>3_000_000_000:self.anchors.popleft()
        if len(self.anchors)>=16 and self.anchors[-1][1]-self.anchors[0][1]>=1_500_000_000:
            fit_anchors = list(self.anchors)[-4:] if self.model is not None else list(self.anchors)
            x=np.array([v[0]-tick for v in fit_anchors]);y=np.array([v[1]-midpoint for v in fit_anchors])
            slope,intercept=np.polyfit(x,y,1)
            if not 20000<slope<30000: raise ValueError('Implausible causal clock slope')
            self.model=(tick,midpoint+intercept,slope)
        elif self.model is not None:
            raise ValueError('Insufficient recent clock observations')

    def predict(self,tick):
        if self.model is None:raise ValueError('Clock warmup incomplete')
        x,y,slope=self.model
        return y+(tick-x)*slope

    def map_sample(self,tick,receipt_ns):
        if self.model is None:return None  # explicitly excluded startup, not replacement timestamps
        if self.last_usable_host is None or receipt_ns-self.last_usable_host>500_000_000:
            raise ValueError('Clock observations stale')
        ns=round(self.predict(tick))
        if self.last_emitted is not None and ns<=self.last_emitted:
            raise ValueError('Causal mapped timestamp reversed')
        if not -1_000_000<=receipt_ns-ns<=1_000_000_000:
            raise ValueError('Mapped sample outside plausible receipt interval')
        self.last_emitted=ns
        return ns
