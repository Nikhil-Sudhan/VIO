#!/usr/bin/env python3
"""Smoke-test native bindings and spline math against an independent implementation."""
import importlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline


def main():
    imported={}
    for name in ['numpy_eigen','sm','aslam_cv','aslam_cameras_april','aslam_backend',
                 'bsplines','aslam_splines','kalibr_common','kalibr_camera_calibration',
                 'kalibr_imu_camera_calibration']:
        imported[name]=importlib.import_module(name).__file__
    import sm
    import bsplines
    rng=np.random.default_rng(682)
    for _ in range(20):
        rvec=rng.normal(size=3)
        q=sm.axisAngle2quat(rvec)
        R=sm.quat2r(q)
        np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R),1,atol=1e-12)
        np.testing.assert_allclose(sm.quat2r(sm.quatInv(q)),R.T,atol=1e-12)
        np.testing.assert_allclose(sm.quat2r(sm.r2quat(R)),R,atol=1e-12)
    comparisons=0
    for order in [3,4,5]:
        spline=bsplines.BSpline(order)
        nknots=spline.numKnotsRequired(8)
        ncoeff=spline.numCoefficientsRequired(8)
        knots=np.linspace(0,1,nknots)
        coeff=rng.normal(size=(3,ncoeff))
        spline.setKnotVectorAndCoefficients(knots,coeff)
        reference=BSpline(knots,coeff.T,order-1)
        for t in np.linspace(knots[order-1]+1e-6,knots[-order]-1e-6,17):
            for derivative in [0,1,2]:
                np.testing.assert_allclose(np.asarray(spline.evalD(t,derivative)).ravel(),
                    reference(t,nu=derivative),rtol=1e-9,atol=1e-9)
                comparisons+=1
    result={'imports':imported,'quaternion_roundtrips':20,
            'spline_value_derivative_comparisons_to_scipy':comparisons,
            'scope':'Binding/numerical smoke checks, not sensor calibration validation'}
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
