#!/usr/bin/env python3
"""Experimental gyro-only rolling-shutter correction for preserved replay images.

Original images and timestamps remain untouched. Readout sign/phase are explicit
hypotheses, not optically measured calibration. Translation and exposure blur
are not corrected. Output remains unsuitable for accepted navigation until
calibration and physical validation succeed.
"""
import argparse,csv,json,hashlib,time
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp

class RotationalDeskew:
    def __init__(self,camera,imu,origin,bias,readout_s):
        self.origin=origin;self.readout=readout_s
        self.dt=float(camera['timeshift_cam_imu'])
        self.Rci=Rotation.from_matrix(np.array(camera['T_cam_imu'])[:3,:3])
        self.width,self.height=camera['resolution']
        fx,fy,cx,cy=camera['intrinsics'];self.fx,self.fy,self.cx,self.cy=fx,fy,cx,cy
        self.K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
        self.D=np.array(camera['distortion_coeffs']);self.k1,self.k2,self.p1,self.p2=self.D
        t=(imu[:,0]-origin)*1e-9;w=imu[:,1:4]-np.asarray(bias)
        if np.any(np.diff(t)<=0):raise ValueError('IMU timestamps must increase')
        if np.max(np.diff(t))>.02:raise ValueError('IMU gap exceeds20ms; no gap filling')
        q=np.empty((len(t),4));R=Rotation.identity();q[0]=R.as_quat()
        for i,dw in enumerate((w[1:]+w[:-1])*.5*np.diff(t)[:,None]):
            R=R*Rotation.from_rotvec(dw);q[i+1]=R.as_quat()
        self.orientation=Slerp(t,Rotation.from_quat(q))
        self.tmin,self.tmax=t[0],t[-1]
        xx,yy=np.meshgrid(np.arange(self.width,dtype=np.float32),np.arange(self.height,dtype=np.float32))
        uv=np.stack([xx,yy],axis=-1)
        xy=cv2.undistortPoints(uv.reshape(-1,1,2),self.K,self.D).reshape(self.height,self.width,2)
        self.rays=np.concatenate([xy,np.ones((self.height,self.width,1),np.float32)],axis=2)
        self.rows=(np.arange(self.height)/(self.height-1)-.5)*readout_s

    def maps(self,ns):
        tref=(int(ns)-self.origin)*1e-9+self.dt
        ts=tref+self.rows
        if min(ts.min(),tref)<self.tmin or max(ts.max(),tref)>self.tmax:
            raise ValueError('No bracketing IMU samples for rolling shutter interval')
        # Map a ray at the calibrated effective frame time into each source row.
        R=(self.Rci*self.orientation(ts).inv()*self.orientation(tref)*self.Rci.inv()).as_matrix()
        y=np.broadcast_to(np.arange(self.height)[:,None],(self.height,self.width))
        for _ in range(2):
            ray=np.einsum('hwij,hwj->hwi',R[np.clip(np.rint(y).astype(int),0,self.height-1)],self.rays)
            x=ray[:,:,0]/ray[:,:,2];v=ray[:,:,1]/ray[:,:,2];r2=x*x+v*v;rad=1+self.k1*r2+self.k2*r2*r2
            mx=self.fx*(x*rad+2*self.p1*x*v+self.p2*(r2+2*x*x))+self.cx
            y=self.fy*(v*rad+self.p1*(r2+2*v*v)+2*self.p2*x*v)+self.cy
        valid=(mx>=0)&(mx<self.width-1)&(y>=0)&(y<self.height-1)&(ray[:,:,2]>0)
        return mx.astype(np.float32),y.astype(np.float32),valid

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('images',type=Path);p.add_argument('imu',type=Path);p.add_argument('imucam',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--readout-ms',type=float,required=True,help='Signed first-to-last output row time, experimental hypothesis')
    p.add_argument('--gyro-bias',type=float,nargs=3,required=True,help='Measured resting bias in native IMU axes, rad/s')
    a=p.parse_args();cv2.setNumThreads(1)
    if not .001<=abs(a.readout_ms)<=50:raise ValueError('Readout must be explicit and plausible')
    rows=list(csv.reader(a.images.read_text().splitlines()[1:]));imu=np.loadtxt(a.imu,delimiter=',',comments='#')
    cam=yaml.safe_load('\n'.join(l for l in a.imucam.read_text().splitlines() if not l.startswith('%')))['cam0']
    if cam['camera_model']!='pinhole' or cam['distortion_model']!='radtan':raise ValueError('Only pinhole/radtan implemented')
    a.output.mkdir();(a.output/'images').mkdir()
    deskew=RotationalDeskew(cam,imu,int(imu[0,0]),a.gyro_bias,a.readout_ms*.001)
    valid_all=np.ones((deskew.height,deskew.width),bool);times=[]
    with (a.output/'images.csv').open('x') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['#t_ns','path0'])
        for i,(ns,path) in enumerate(rows):
            begun=time.monotonic();im=cv2.imread(path,0)
            if im is None or im.shape!=valid_all.shape:raise ValueError('Missing/wrong-size input image')
            mx,my,valid=deskew.maps(int(ns));valid_all &= valid
            out=cv2.remap(im,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
            dest=(a.output/'images'/f'{i:06d}.pgm').resolve()
            if not cv2.imwrite(str(dest),out):raise OSError('Image write failed')
            w.writerow([ns,str(dest)]);times.append((time.monotonic()-begun)*1000)
            if i%100==0:print(f'corrected={i+1}/{len(rows)}',flush=True)
    # OpenVINS masks nonzero pixels. Exclude the union of invalid remap borders.
    mask=np.where(valid_all,0,255).astype(np.uint8);cv2.imwrite(str(a.output/'invalid-mask.png'),mask)
    report={'scope':'Experimental offline rotational deskew; NOT accepted calibration or VIO','frames':len(rows),'readout_ms_hypothesis':a.readout_ms,'effective_camera_to_imu_offset_s':deskew.dt,'gyro_bias_rad_s':a.gyro_bias,'timestamp_handling':'Input image timestamps unchanged; calibrated effective frame time used as rotation reference, not relabeled exposure midpoint','limitations':['Readout sign/phase not optically established','Translation and motion blur uncorrected','Fixed bias and extrinsic rotation','Two source-row fixed-point iterations with nearest-row rotation lookup'],'masked_fraction':float((~valid_all).mean()),'processing_ms_p50_p95_max':np.percentile(times,[50,95,100]).tolist(),'inputs_sha256':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [a.images,a.imu,a.imucam]}}
    (a.output/'deskew-report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()
