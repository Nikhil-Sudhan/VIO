"""Fault handling at the boundary between physical acquisition and live transport."""
import sys
import struct
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from online_imu import FifoDecoder,CausalClock
from test_fifo_decode import slot,batch,word
from analyse_recording import decode


class OnlineTests(unittest.TestCase):
    def test_every_word_partition_matches_offline(self):
        payload=b''.join(slot(100+i*192,i%4) for i in range(20))
        for words in [1,2,4,5,7,11]:
            records=[batch(payload[i:i+words*7]) for i in range(0,len(payload),words*7)]
            reference,_,_,faults,trimmed=decode(records)
            self.assertFalse(faults);self.assertFalse(trimmed)
            d=FifoDecoder();actual=[]
            for r in records:actual.extend(d.push(r))
            tail,trimmed=d.finish();actual.extend(tail)
            self.assertFalse(trimmed);self.assertEqual(reference,actual)

    def test_gap_not_hidden_by_counter_wrap(self):
        d=FifoDecoder()
        d.push(batch(slot(100,0)+slot(1060,1)))
        with self.assertRaisesRegex(ValueError,'discontinuity'):d.finish()

    def test_incomplete_interior_is_fatal(self):
        with self.assertRaisesRegex(ValueError,'Incomplete'):
            FifoDecoder().push(batch(slot(100,0)[:14]+slot(292,1)))

    def test_nominal_gyro_range_distinct_from_adc_clipping(self):
        payload=slot(100,0)
        payload=payload[:7]+word(1,0,struct.pack('<hhh',29000,0,0))+payload[14:]
        d=FifoDecoder();d.push(batch(payload))
        with self.assertRaisesRegex(ValueError,'nominal range'):d.finish()

    def test_counter_rollover(self):
        d=FifoDecoder();groups=d.push(batch(slot(2**32-100,3)+slot(92,0)));tail,trimmed=d.finish()
        self.assertFalse(trimmed);self.assertEqual((groups+tail)[1]['ticks']-(groups+tail)[0]['ticks'],192)

    def test_clock_prior_predictions_and_staleness(self):
        c=CausalClock();mapped=[]
        for i in range(100):
            tick=4000*i+1000;expected=10_000_000_000+tick*25520
            c.anchor({'sensor_ticks_u32':tick,'host_before_boot_ns':expected-200_000,'host_after_boot_ns':expected+200_000})
            ns=c.map_sample(tick,expected+1_000_000)
            if ns is not None:mapped.append(ns);self.assertLess(abs(ns-expected),2)
        self.assertGreater(len(mapped),50);self.assertGreater(len(c.validation_residuals),10)
        self.assertTrue(np.all(np.diff(mapped)>0))
        with self.assertRaisesRegex(ValueError,'stale'):c.map_sample(tick+40000,expected+1_000_000_000)

    def test_clock_reset_rejected(self):
        c=CausalClock()
        c.anchor({'sensor_ticks_u32':4000,'host_before_boot_ns':1,'host_after_boot_ns':20})
        with self.assertRaisesRegex(ValueError,'reversed'):
            c.anchor({'sensor_ticks_u32':3000,'host_before_boot_ns':21,'host_after_boot_ns':40})

    def test_twenty_hz_anchors_tolerate_wide_read_rejections(self):
        c=CausalClock();mapped=[]
        for i in range(240):
            tick=2000*i+1000;host=10_000_000_000+tick*25520
            # Every third observation is deliberately unusable; no tolerance
            # is increased. The remaining anchors must support the model.
            half_width=2_000_000 if i%3==0 else 400_000
            c.anchor({'sensor_ticks_u32':tick,'host_before_boot_ns':host-half_width,
                      'host_after_boot_ns':host+half_width})
            ns=c.map_sample(tick,host+3_000_000)
            if ns is not None:
                self.assertLess(abs(ns-host),2);mapped.append(ns)
        self.assertGreater(len(mapped),150)
        self.assertTrue(np.all(np.diff(mapped)>0))
        self.assertGreater(len(c.validation_residuals),20)
        with self.assertRaisesRegex(ValueError,'stale'):
            c.map_sample(tick+40000,host+1_000_000_000)

    def test_clock_tracks_observed_half_percent_rate_change(self):
        c=CausalClock();host=10_000_000_000;last=None
        for i in range(120):
            host += 4000 * (25520 if i < 80 else 25650)
            tick=1000+i*4000
            c.anchor({'sensor_ticks_u32':tick,'host_before_boot_ns':host-400_000,'host_after_boot_ns':host+400_000})
            ns=c.map_sample(tick,host+1_000_000)
            if ns is not None:
                if last is not None:self.assertGreater(ns,last)
                self.assertLess(abs(ns-host),1_000_000)
                last=ns
        self.assertLess(max(map(abs,c.validation_residuals)),1_000_000)
        # A discontinuous clock jump still fails the original error limit.
        with self.assertRaisesRegex(ValueError,'exceeds 1 ms'):
            c.anchor({'sensor_ticks_u32':tick+4000,'host_before_boot_ns':host+4000*25650+5_000_000-400_000,
                      'host_after_boot_ns':host+4000*25650+5_000_000+400_000})

    def test_clock_follows_one_percent_rate_change_at_higher_anchor_rate(self):
        c=CausalClock();host=10_000_000_000;last=None
        for i in range(160):
            host += 2000 * (25520 if i<100 else 25265)
            tick=1000+i*2000
            c.anchor({'sensor_ticks_u32':tick,'host_before_boot_ns':host-400_000,
                      'host_after_boot_ns':host+400_000})
            ns=c.map_sample(tick,host+2_000_000)
            if ns is not None:
                if last is not None:self.assertGreater(ns,last)
                self.assertLess(abs(ns-host),1_000_000);last=ns
        self.assertLess(max(map(abs,c.validation_residuals)),1_000_000)


if __name__=='__main__':unittest.main()
