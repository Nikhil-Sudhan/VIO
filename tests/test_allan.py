import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from imu_noise import overlapping_adev


class AllanTests(unittest.TestCase):
    def test_white_noise_density(self):
        dt=.005;density=.02;rng=np.random.default_rng(72)
        data=rng.normal(size=200000)*density/np.sqrt(dt)
        sizes=np.array([10,20,40,80])
        actual=overlapping_adev(data,sizes).ravel()*np.sqrt(sizes*dt)
        np.testing.assert_allclose(actual,density,rtol=.06)

    def test_random_walk_strength(self):
        dt=.005;strength=.001;rng=np.random.default_rng(73)
        data=np.cumsum(rng.normal(size=200000))*strength*np.sqrt(dt)
        sizes=np.array([20,40,80,160])
        actual=overlapping_adev(data,sizes).ravel()*np.sqrt(3/(sizes*dt))
        np.testing.assert_allclose(actual,strength,rtol=.08)

    def test_constant_offset_does_not_change_deviation(self):
        values=np.arange(3000)*.001
        np.testing.assert_allclose(overlapping_adev(values,[10,100]),overlapping_adev(values+9.81,[10,100]),atol=1e-10)


if __name__=='__main__':unittest.main()
