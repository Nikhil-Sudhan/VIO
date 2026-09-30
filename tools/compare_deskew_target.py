#!/usr/bin/env python3
"""Compare paired AprilGrid reprojection residuals on raw and corrected images."""
import argparse,csv,json
from pathlib import Path
import cv2,numpy as np,yaml
from calibrate_intrinsics import detect

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('original',type=Path);p.add_argument('corrected',type=Path);p.add_argument('camera',type=Path);p.add_argument('output',type=Path);p.add_argument('--stride',type=int,default=10);a=p.parse_args();cv2.setNumThreads(1)
 cam=yaml.safe_load('\n'.join(l for l in a.camera.read_text().splitlines() if not l.startswith('%')))['cam0'];fx,fy,cx,cy=cam['intrinsics'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
 D=np.array(cam['distortion_coeffs']);params=cv2.aruco.DetectorParameters();params.markerBorderBits=2;params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_APRILTAG;det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
 raw=dict(csv.reader(a.original.read_text().splitlines()[1:]));corrected=list(csv.reader(a.corrected.read_text().splitlines()[1:]));result=[]
 for ns,path in corrected[::a.stride]:
  first=detect(raw[ns],det);second=detect(path,det)
  if first is None or second is None:continue
  ids=sorted(set(first['ids'])&set(second['ids']))
  if len(ids)<10:continue
  rms=[]
  for view in [first,second]:
   indices=np.concatenate([np.arange(4*view['ids'].index(i),4*view['ids'].index(i)+4) for i in ids]);obj=view['object'][indices];uv=view['image'][indices]
   ok,rv,tv=cv2.solvePnP(obj,uv,K,D)
   if not ok:raise RuntimeError('Pose fit failed')
   pred,_=cv2.projectPoints(obj,rv,tv,K,D);rms.append(float(np.sqrt(np.mean(np.sum((pred.reshape(-1,2)-uv)**2,axis=1)))))
  result.append({'timestamp_ns':int(ns),'common_tags':len(ids),'raw_rms_px':rms[0],'corrected_rms_px':rms[1]})
 report={'scope':'Paired target reprojection comparison, not independent pose truth or VIO accuracy','sample_stride':a.stride,'paired_frames':len(result),'samples':result}
 if result:
  x=np.array([[v['raw_rms_px'],v['corrected_rms_px']] for v in result]);report.update(median_raw_corrected_px=np.median(x,axis=0).tolist(),p95_raw_corrected_px=np.percentile(x,95,axis=0).tolist(),fraction_improved=float(np.mean(x[:,1]<x[:,0])))
 a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
if __name__=='__main__':main()
