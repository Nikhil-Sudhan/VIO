"""Analytic geometry checks for the optional offline deskew, not hardware accuracy."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from deskew_replay import RotationalDeskew

class DeskewTests(unittest.TestCase):
    def camera(self):
        return dict(intrinsics=[100.,100.,40.,29.5],resolution=[80,60],distortion_coeffs=[0.,0.,0.,0.],T_cam_imu=np.eye(4).tolist(),timeshift_cam_imu=0.)
    def imu(self,wy):
        a=np.zeros((101,7));a[:,0]=np.arange(101)*10000000;a[:,2]=wy;return a
    def test_stationary_camera_preserves_coordinates(self):
        d=RotationalDeskew(self.camera(),self.imu(0),0,[0,0,0],.02)
        x,y,_=d.maps(500000000);xx,yy=np.meshgrid(np.arange(80),np.arange(60))
        np.testing.assert_allclose(x,xx,atol=1e-5);np.testing.assert_allclose(y,yy,atol=1e-5)
    def test_constant_yaw_has_opposite_top_and_bottom_displacement(self):
        d=RotationalDeskew(self.camera(),self.imu(1),0,[0,0,0],.02)
        x,y,_=d.maps(500000000)
        self.assertAlmostEqual(float(x[0,40]),40+100*np.tan(.01),places=4)
        self.assertAlmostEqual(float(x[-1,40]),40-100*np.tan(.01),places=4)
    def test_no_imu_extrapolation(self):
        d=RotationalDeskew(self.camera(),self.imu(1),0,[0,0,0],.02)
        with self.assertRaisesRegex(ValueError,'bracketing'):d.maps(0)

if __name__=='__main__':unittest.main()
