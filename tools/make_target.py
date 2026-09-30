#!/usr/bin/env python3
"""Render Kalibr-compatible t36h11 codes, two-bit border, onto A4 vector PDF.

Codes and layout follow Kalibr's kalibr_create_target_pdf; local source digest
is in config/april36h11-first36.json. Dimensions are NOMINAL until print measured.
"""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import cv2

root=Path(__file__).resolve().parents[1]
out=root/'calibration/target'
out.mkdir(parents=True,exist_ok=True)
codes=json.loads((root/'config/april36h11-first36.json').read_text())['codes']
fig=plt.figure(figsize=(210/25.4,297/25.4))
ax=fig.add_axes([0,0,1,1]); ax.set(xlim=(0,210),ylim=(0,297)); ax.set_aspect('equal'); ax.axis('off')
size,space=20.,6.
for tag,code in enumerate(codes):
    x,y=30+(tag%6)*26,75+(tag//6)*26
    ax.add_patch(Rectangle((x,y),size,size,color='black',linewidth=0))
    # Kalibr rotates its bit matrix twice, then draws rows top down.
    bits=np.rot90(np.array([[(code>>(6*i+j))&1 for j in range(6)] for i in range(6)]),2)
    for i in range(6):
        for j in range(6):
            if bits[i,j]:
                ax.add_patch(Rectangle((x+(j+2)*2,y+(7-i)*2),2,2,color='white',linewidth=0))
    for cx,cy in ((x-space,y-space),(x+size,y-space),(x+size,y+size),(x-space,y+size)):
        ax.add_patch(Rectangle((cx,cy),space,space,color='black',linewidth=0))
ax.text(105,266,'VIO calibration target — A4',ha='center',fontsize=15)
ax.text(105,256,'Kalibr AprilGrid 6 x 6 | t36h11 | 2-bit border',ha='center',fontsize=10)
ax.text(105,45,'Print at 100% / Actual size. Disable Fit to page.',ha='center',fontsize=10)
ax.text(105,38,'Nominal black tag edge: 20 mm; gap: 6 mm.',ha='center',fontsize=9)
ax.plot([55,155],[25,25],color='black',linewidth=.7)
ax.plot([55,55],[23,27],color='black',linewidth=.7)
ax.plot([155,155],[23,27],color='black',linewidth=.7)
ax.text(105,17,'Measure this line: nominal 100 mm',ha='center',fontsize=9)
fig.savefig(out/'aprilgrid-a4.pdf')
fig.savefig(out/'aprilgrid-a4.png',dpi=200)
plt.close(fig)
image=cv2.imread(str(out/'aprilgrid-a4.png'),0)
params=cv2.aruco.DetectorParameters(); params.markerBorderBits=2
detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params)
corners,ids,_=detector.detectMarkers(image)
found=[] if ids is None else sorted(ids.ravel().tolist())
report={'nominal_tag_size_m':.020,'nominal_spacing_ratio':.3,'printed_dimensions_verified':False,
        'digital_render_decoded_ids':found,'all_36_ids_detected':found==list(range(36))}
(out/'render-check.json').write_text(json.dumps(report,indent=2)+'\n')
(out/'target-NOMINAL.yaml').write_text("# NOT MEASURED: verify printed tag dimensions before calibration.\ntarget_type: aprilgrid\ntagCols: 6\ntagRows: 6\ntagSize: 0.020\ntagSpacing: 0.3\n")
print(json.dumps(report,indent=2))
if not report['all_36_ids_detected']:
    raise SystemExit('Digital target validation failed; do not print yet')
