import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from clock_mapping import fit_clock


class ClockTests(unittest.TestCase):
    def observations(self):
        x=np.arange(500,dtype=float)*4000
        # Synthetic changing clock rate plus sub-bracket observation jitter.
        y=1e12+x*25520+1e-6*x*x
        jitter=15000*np.sin(np.arange(len(x)))
        anchors=[{'sensor_ticks_u32':int(xx),'host_before_boot_ns':yy+jj-400000,
                  'host_after_boot_ns':yy+jj+400000} for xx,yy,jj in zip(x,y,jitter)]
        return x,y,anchors

    def test_drift_and_withheld_observations(self):
        x,y,a=self.observations();f,info,_,_,_=fit_clock(a)
        self.assertGreater(info['anchors_withheld'],50)
        self.assertLess(np.max(abs(f(x)-y)),30000)
        self.assertTrue(np.all(np.diff(f(x))>0))
        self.assertGreater(info['global_affine_diagnostic']['withheld_residual_p95_us'],100)

    def test_long_receipt_stall_is_excluded(self):
        x,y,a=self.observations();a[100]['host_after_boot_ns']+=100000000
        f,info,_,_,_=fit_clock(a)
        self.assertLess(abs(f(x[100])-y[100]),30000)
        self.assertLess(info['anchors_fitted']+info['anchors_withheld'],len(a))

    def test_clock_reset_rejected(self):
        _,_,a=self.observations();a[200]['sensor_ticks_u32']=0
        with self.assertRaises(ValueError):fit_clock(a)

    def test_boundary_stalls_use_full_observation_window(self):
        x,y,a=self.observations()
        # Startup/shutdown bus delays leave fewer than five good anchors in
        # the old half-width endpoint window, but enough in a full window.
        for i in list(range(7))+list(range(len(a)-7,len(a))):
            a[i]['host_after_boot_ns']+=10_000_000
        f,info,_,_,_=fit_clock(a)
        self.assertLess(np.max(abs(f(x)-y)),30000)
        self.assertTrue(np.all(np.diff(f(x))>0))
        self.assertEqual(info['anchor_read_width_cutoff_ns'],2_000_000)

    def test_observation_gap_rejected(self):
        _,_,a=self.observations();a=a[:100]+a[180:]
        with self.assertRaises(ValueError):fit_clock(a)


if __name__=='__main__':unittest.main()
