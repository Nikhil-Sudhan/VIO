"""Bounded acquisition-to-OpenVINS transport; no hardware devices opened here."""
import math
import json
import queue
import re
import shutil
import subprocess
import tempfile
import threading
import time
import traceback
from pathlib import Path
from online_imu import CausalClock,FifoDecoder
from stream_protocol import MAGIC,imu_packet,camera_packet,end_packet

ROOT=Path(__file__).resolve().parents[1]


class LiveBridge:
    def __init__(self,config,folder,stop_event,errors,ram_output=False):
        self.folder=Path(folder)
        self.scratch=None
        self.events=queue.Queue(maxsize=256)
        self.stop=stop_event;self.errors=errors
        self.decoder=FifoDecoder();self.clock=CausalClock()
        self.report={'scope':'Experimental physical stream; output requires separate validation',
            'imu_sent':0,'camera_sent':0,'clock_warmup_imu_excluded':0,'clock_warmup_camera_excluded':0,
            'event_queue_high_water':0,'normal_end_sent':False}
        self.report.update(max_event_queue_wait_ms=0.0,max_event_processing_ms=0.0,
                           max_camera_write_ms=0.0,max_imu_group_write_ms=0.0)
        output_folder=self.folder
        if ram_output:
            scratch_root=Path('/dev/shm')
            if shutil.disk_usage(scratch_root).free < 256*1024*1024:
                raise RuntimeError('Insufficient RAM filesystem space for estimator output')
            self.scratch=Path(tempfile.mkdtemp(prefix='pi-vio-',dir=scratch_root))
            output_folder=self.scratch
            (self.folder/'vio').symlink_to(self.scratch/'vio',target_is_directory=True)
            (self.folder/'estimator.log').symlink_to(self.scratch/'estimator.log')
            self.report['derived_output_storage']='RAM during live processing; copied to the recorder session at shutdown'
            self.report['raw_recording_destination']=str(self.folder)
            self.report['derived_output_persisted']=False
            (self.folder/'vio-storage.json').write_text(json.dumps({'scratch':str(self.scratch),'persisted':False,
                'scope':'Derived estimator output only. Raw measurements use the recorder session directory.'},indent=2)+'\n')
        self.console_path=output_folder/'estimator.log'
        self.console=self.console_path.open('xb')
        self.command=[str(ROOT/'build/adapter/vio_stream'),str(Path(config).resolve()),str(output_folder/'vio'),'live']
        self.child=None
        self.thread=threading.Thread(target=self.run,name='estimator-transport',daemon=True)
        self.thread.start()

    def submit(self,event):
        self.events.put_nowait((time.monotonic_ns(),event))
        self.report['event_queue_high_water']=max(self.report['event_queue_high_water'],self.events.qsize())

    def emit_groups(self,groups):
        for group in groups:
            ns=self.clock.map_sample(group['ticks'],group['receipt_ns'])
            if ns is None:
                self.report['clock_warmup_imu_excluded']+=1;continue
            values=[v*math.pi/180*.0175 for v in group['gyro']]+[v*9.80665*.000122 for v in group['accel']]
            if self.child is None:
                self.child=subprocess.Popen(self.command,stdin=subprocess.PIPE,stdout=self.console,stderr=subprocess.STDOUT,cwd=ROOT)
                self.child.stdin.write(MAGIC)
            write_started=time.monotonic_ns()
            self.child.stdin.write(imu_packet(ns,group['receipt_ns'],self.report['imu_sent'],values))
            self.report['max_imu_group_write_ms']=max(self.report['max_imu_group_write_ms'],(time.monotonic_ns()-write_started)/1e6)
            self.report['imu_sent']+=1

    def run(self):
        try:
            while True:
                item=self.events.get()
                if item is None:break
                queued,item=item
                started=time.monotonic_ns()
                self.report['max_event_queue_wait_ms']=max(self.report['max_event_queue_wait_ms'],(started-queued)/1e6)
                if item['kind']=='clock_anchor':self.clock.anchor(item)
                elif item['kind']=='fifo':self.emit_groups(self.decoder.push(item))
                elif item['kind']=='camera':
                    if not self.report['imu_sent']:
                        self.report['clock_warmup_camera_excluded']+=1;continue
                    write_started=time.monotonic_ns()
                    self.child.stdin.write(camera_packet(item['timestamp_ns'],item['receipt_ns'],item['sequence'],item['pixels']))
                    self.report['max_camera_write_ms']=max(self.report['max_camera_write_ms'],(time.monotonic_ns()-write_started)/1e6)
                    self.report['camera_sent']+=1
                else:raise ValueError('Unknown acquisition event')
                if self.child is not None:self.child.stdin.flush()
                self.report['max_event_processing_ms']=max(self.report['max_event_processing_ms'],(time.monotonic_ns()-started)/1e6)
            groups,trimmed=self.decoder.finish();self.emit_groups(groups)
            self.report['trailing_incomplete_slot_trimmed']=trimmed
            if self.child is None:raise RuntimeError('No usable IMU stream after clock warmup')
            self.child.stdin.write(end_packet());self.child.stdin.close();self.report['normal_end_sent']=True
            code=self.child.wait(timeout=10);self.report['estimator_exit_code']=code
            if code:raise RuntimeError(f'Estimator failed with exit code {code}; see estimator.log')
        except Exception as exc:
            self.errors.append(traceback.format_exc());self.stop.set()
            # A broken pipe commonly means the estimator already reported a
            # specific failure. Give that exit a short chance to finish first.
            if isinstance(exc,BrokenPipeError) and self.child is not None:
                try:self.child.wait(timeout=.25)
                except subprocess.TimeoutExpired:pass
            if self.child is not None and self.child.poll() is None:
                self.child.terminate()
                try:self.child.wait(timeout=5)
                except subprocess.TimeoutExpired:self.child.kill();self.child.wait(timeout=5)
        finally:
            if self.child is not None:
                self.report['estimator_exit_code']=self.child.poll()
                try:self.child.stdin.close()
                except OSError:pass  # terminal broken pipe was already recorded
            self.console.close()
            if self.child is not None and self.report['estimator_exit_code'] not in (None,0):
                with self.console_path.open('rb') as log:
                    log.seek(0,2);size=log.tell();log.seek(max(0,size-4096))
                    tail=re.sub(r'\x1b\[[0-9;]*m','',log.read().decode(errors='replace'))[-2000:]
                self.report['estimator_error_tail']=tail
                self.errors.append(f"Estimator exit {self.report['estimator_exit_code']}; estimator.log tail:\n{tail}")

    def close(self):
        if self.thread.is_alive():
            try:self.events.put(None,timeout=5)
            except queue.Full:
                self.errors.append('Estimator event queue would not drain at shutdown')
                if self.child is not None:self.child.terminate()
        self.thread.join(timeout=15)
        if self.thread.is_alive():
            if self.child is not None:self.child.terminate()
            self.errors.append('Estimator transport did not finish; output incomplete')
            self.stop.set()
        self.report['clock_prediction_checks']=len(self.clock.validation_residuals)
        self.report['clock_prediction_max_abs_us']=max(map(abs,self.clock.validation_residuals),default=0)/1000
        if self.scratch is not None:
            if self.thread.is_alive():
                raise RuntimeError('Cannot persist estimator output while transport is still running; RAM files retained')
            # Copy only after the estimator exits. Live sensor capture and the
            # filter never wait for these SD-card output writes.
            if (self.scratch/'vio').is_dir():
                temporary=self.folder/'.vio-saving'
                shutil.copytree(self.scratch/'vio',temporary)
                (self.folder/'vio').unlink()
                temporary.rename(self.folder/'vio')
            else:
                (self.folder/'vio').unlink()
            temporary=self.folder/'estimator.log.saving'
            shutil.copy2(self.console_path,temporary)
            temporary.replace(self.folder/'estimator.log')
            self.report['derived_output_persisted']=True
            (self.folder/'vio-storage.json').write_text(json.dumps({'persisted':True,
                'scope':'Derived output copied from RAM to the recorder session after estimator shutdown.'},indent=2)+'\n')
            shutil.rmtree(self.scratch)
            self.scratch=None
        return self.report
