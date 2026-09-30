"""Transport startup/drain tests with a byte sink, never a physical VIO result."""
import io
import json
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from live_bridge import LiveBridge
from stream_protocol import MAGIC,HEADER,IMU
from test_fifo_decode import slot,batch


class KeepBytes(io.BytesIO):
    def close(self):self.flush()


class Sink:
    def __init__(self,*args,**kwargs):self.stdin=KeepBytes();self.returncode=None
    def wait(self,timeout=None):self.returncode=0;return 0
    def poll(self):return self.returncode
    def terminate(self):self.returncode=-15
    def kill(self):self.returncode=-9


class BridgeTests(unittest.TestCase):
    def test_ram_outputs_survive_estimator_failure_on_disk(self):
        class BrokenInput(io.BytesIO):
            def write(self,data):raise BrokenPipeError('estimator exited')
        class FailedSink(Sink):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.stdin=BrokenInput();self.returncode=17
                output=Path(args[0][2]);output.mkdir()
                (output/'poses.csv').write_text('original saved estimator output\n')
                kwargs['stdout'].write(b'ERROR: recorded failure\n');kwargs['stdout'].flush()
            def wait(self,timeout=None):return self.returncode
        errors=[];stop=threading.Event()
        with tempfile.TemporaryDirectory() as folder,patch('live_bridge.subprocess.Popen',FailedSink), \
                patch('live_bridge.CausalClock.map_sample',return_value=1_000_000_000):
            bridge=LiveBridge('unused.yaml',folder,stop,errors,ram_output=True)
            scratch=bridge.scratch
            bridge.submit(batch(slot(100,0)+slot(292,1)))
            result=bridge.close()
            folder=Path(folder)
            self.assertFalse((folder/'vio').is_symlink())
            self.assertFalse((folder/'estimator.log').is_symlink())
            self.assertEqual((folder/'vio/poses.csv').read_text(),'original saved estimator output\n')
            self.assertIn('recorded failure',(folder/'estimator.log').read_text())
            self.assertTrue(json.loads((folder/'vio-storage.json').read_text())['persisted'])
            self.assertFalse(scratch.exists())
        self.assertTrue(stop.is_set())
        self.assertEqual(result['estimator_exit_code'],17)
        self.assertTrue(result['derived_output_persisted'])

    def test_estimator_failure_preserves_reason_and_exit_code(self):
        class BrokenInput(io.BytesIO):
            def write(self,data):raise BrokenPipeError('estimator exited')
        class FailedSink(Sink):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.stdin=BrokenInput();self.returncode=17
                kwargs['stdout'].write(b'ERROR: example estimator covariance failure\n')
                kwargs['stdout'].flush()
            def wait(self,timeout=None):return self.returncode
        errors=[];stop=threading.Event()
        with tempfile.TemporaryDirectory() as folder,patch('live_bridge.subprocess.Popen',FailedSink), \
                patch('live_bridge.CausalClock.map_sample',return_value=1_000_000_000):
            bridge=LiveBridge('unused.yaml',folder,stop,errors)
            bridge.submit(batch(slot(100,0)+slot(292,1)))
            result=bridge.close()
        self.assertTrue(stop.is_set())
        self.assertEqual(result['estimator_exit_code'],17)
        self.assertIn('example estimator covariance failure',result['estimator_error_tail'])
        self.assertTrue(any('Estimator exit 17' in e for e in errors))
        self.assertFalse(result['normal_end_sent'])

    def test_warmup_then_fifo_bytes_and_normal_drain(self):
        stop=threading.Event();errors=[]
        with tempfile.TemporaryDirectory() as folder,patch('live_bridge.subprocess.Popen',Sink):
            bridge=LiveBridge('unused.yaml',folder,stop,errors)
            bridge.submit({'kind':'camera','timestamp_ns':1,'receipt_ns':2,'sequence':0,'pixels':np.zeros((2,2),np.uint8)})
            for i in range(30):
                tick=100+i*24*192;host=10_000_000_000+tick*25520
                bridge.submit({'kind':'clock_anchor','sensor_ticks_u32':tick,'host_before_boot_ns':host-200_000,'host_after_boot_ns':host+200_000})
                r=batch(b''.join(slot(tick+j*192,j%4) for j in range(24)))
                r['host_after_boot_ns']=host+23*192*25520+2_000_000
                bridge.submit(r)
            result=bridge.close()
            self.assertFalse(errors);self.assertFalse(stop.is_set())
            self.assertTrue(result['normal_end_sent']);self.assertEqual(result['clock_warmup_camera_excluded'],1)
            self.assertGreater(result['imu_sent'],100);self.assertGreater(result['clock_warmup_imu_excluded'],100)
            packet=bridge.child.stdin.getvalue();self.assertTrue(packet.startswith(MAGIC))
            offset=8;last_ns=None;count=0
            while offset<len(packet):
                kind,ns,receipt,seq,n=HEADER.unpack_from(packet,offset);offset+=HEADER.size
                body=packet[offset:offset+n];offset+=n
                if kind==3:
                    self.assertEqual(n,0);self.assertEqual(offset,len(packet));break
                self.assertEqual(kind,1);self.assertEqual(seq,count);count+=1
                if last_ns is not None:self.assertGreater(ns,last_ns)
                last_ns=ns
                self.assertGreater(receipt,ns)
                values=IMU.unpack(body)
                self.assertAlmostEqual(values[0],-np.pi/180*.0175)
                self.assertAlmostEqual(values[5],8192*9.80665*.000122)
            self.assertEqual(count,result['imu_sent'])

    def test_short_stream_does_not_launch_estimator(self):
        errors=[];stop=threading.Event()
        with tempfile.TemporaryDirectory() as folder,patch('live_bridge.subprocess.Popen') as launch:
            bridge=LiveBridge('unused.yaml',folder,stop,errors);bridge.close()
            launch.assert_not_called()
        self.assertTrue(stop.is_set());self.assertTrue(any('No usable IMU' in e for e in errors))


if __name__=='__main__':unittest.main()
