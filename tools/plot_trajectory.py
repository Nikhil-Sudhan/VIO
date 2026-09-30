#!/usr/bin/env python3
"""Save a standalone trajectory figure and interactive local HTML viewer."""
import argparse
import html
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    p.add_argument('--title',required=True);a=p.parse_args()
    poses=np.genfromtxt(a.run/'poses.csv',delimiter=',',names=True)
    if poses.ndim!=1 or len(poses)<2:raise ValueError('Need actual pose logs')
    aligned=a.run/'aligned_reference.csv'
    if aligned.exists():
        d=np.genfromtxt(aligned,delimiter=',',names=True)
        xyz=np.column_stack([d[n] for n in ['px','py','pz']]);t=d['t_rel_s']
        gt=np.column_stack([d[n] for n in ['gt_x','gt_y','gt_z']]);note='Rigid alignment, metric scale fixed at 1. Reference dataset only.'
    else:
        xyz=np.column_stack([poses[n] for n in ['px','py','pz']]);t=poses['t_rel_s'];gt=None
        note='Local IMU trajectory; no independent ground truth included.'
    speed=np.sqrt(sum(poses[n]**2 for n in ('vx','vy','vz')))
    fig,axes=plt.subplots(2,2,figsize=(11,8),layout='constrained')
    for ax,(i,j) in zip(axes.flat[:3],[(0,1),(0,2),(1,2)]):
        if gt is not None:ax.plot(gt[:,i],gt[:,j],color='#e89b28',lw=2,label='Ground truth')
        ax.plot(xyz[:,i],xyz[:,j],color='#1675b8',lw=1,label='OpenVINS')
        ax.scatter(xyz[0,i],xyz[0,j],color='#18894d',s=30,label='First estimate',zorder=3)
        ax.set(xlabel=f'{"XYZ"[i]} (m)',ylabel=f'{"XYZ"[j]} (m)');ax.axis('equal');ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8)
    if gt is not None:
        axes[1,1].plot(t,d['error_m'],color='#1675b8');axes[1,1].set_ylabel('Position error (m)')
    else:
        axes[1,1].plot(poses['t_rel_s'],speed);axes[1,1].set_ylabel('Speed (m/s)')
    axes[1,1].set_xlabel('Time relative to logged origin (s)');axes[1,1].grid(alpha=.25)
    fig.suptitle(a.title+'\n'+note,fontsize=12)
    fig.savefig(a.run/'trajectory.png',dpi=160);fig.savefig(a.run/'trajectory.pdf');plt.close(fig)
    data={'xyz':xyz.tolist(),'time':t.tolist(),'gt':None if gt is None else gt.tolist(),
          'speed':np.interp(t,poses['t_rel_s'],speed).tolist()}
    template='''<!doctype html><meta charset="utf-8"><title>Trajectory viewer</title>
<style>body{font:16px system-ui;background:#101820;color:#e5edf5;max-width:960px;margin:30px auto;padding:16px}h1{font-size:24px}canvas{width:100%;background:#18232f;border-radius:8px}input{width:65%;vertical-align:middle}select,button{font:inherit;margin:8px;padding:5px}p{line-height:1.5}.blue{color:#62b9f0}.orange{color:#ffba59}</style>
<h1>__TITLE__</h1><p>__NOTE__</p><p><span class="blue">OpenVINS</span> · <span class="orange">Ground truth (when available)</span> · green: first estimate</p>
<label>Projection <select id="projection"><option value="0,1">XY</option><option value="0,2">XZ</option><option value="1,2">YZ</option></select></label>
<button id="play">Play</button><input id="cursor" type="range" min="0" value="0"><output id="label"></output>
<canvas id="plot" width="960" height="640"></canvas>
<p>Coordinates and speed refer to the IMU origin. Time is relative to the integer nanosecond origin in run.txt. This viewer reads only embedded recorded output; it does not run VIO.</p>
<script>const data=__DATA__;
const canvas=document.getElementById('plot'),ctx=canvas.getContext('2d'),slider=document.getElementById('cursor'),proj=document.getElementById('projection');slider.max=data.xyz.length-1;
function draw(){const [i,j]=proj.value.split(',').map(Number),k=+slider.value,all=data.xyz.concat(data.gt||[]);let lo=[Infinity,Infinity],hi=[-Infinity,-Infinity];for(const p of all){lo[0]=Math.min(lo[0],p[i]);hi[0]=Math.max(hi[0],p[i]);lo[1]=Math.min(lo[1],p[j]);hi[1]=Math.max(hi[1],p[j]);}const scale=Math.min(840/Math.max(hi[0]-lo[0],.01),510/Math.max(hi[1]-lo[1],.01));const mx=(lo[0]+hi[0])/2,my=(lo[1]+hi[1])/2;function point(p){return[480+(p[i]-mx)*scale,310-(p[j]-my)*scale]}ctx.clearRect(0,0,960,640);ctx.font='16px system-ui';ctx.fillStyle='#d0deed';ctx.fillText('XYZ'[i]+' (m) →',805,614);ctx.fillText('XYZ'[j]+' (m) ↑',20,30);ctx.fillText('Equal axis scale · '+(100/scale).toFixed(2)+' m per 100 px',20,614);function path(points,color,end){ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;points.slice(0,end).forEach((p,n)=>{const[x,y]=point(p);if(n===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke()}if(data.gt)path(data.gt,'#ffba59',data.gt.length);path(data.xyz,'#62b9f0',k+1);for(const[n,color]of[[0,'#3bc887'],[k,'#e5edf5']]){const[x,y]=point(data.xyz[n]);ctx.beginPath();ctx.fillStyle=color;ctx.arc(x,y,5,0,Math.PI*2);ctx.fill()}document.getElementById('label').textContent=data.time[k].toFixed(2)+' s · '+data.speed[k].toFixed(2)+' m/s';}
slider.oninput=draw;proj.onchange=draw;let timer=null;document.getElementById('play').onclick=function(){if(timer){clearInterval(timer);timer=null;this.textContent='Play'}else{this.textContent='Pause';timer=setInterval(()=>{slider.value=(+slider.value+1)%data.xyz.length;draw()},50)}};slider.value=slider.max;draw();</script>'''
    template=template.replace('__TITLE__',html.escape(a.title)).replace('__NOTE__',html.escape(note)).replace('__DATA__',json.dumps(data,allow_nan=False))
    (a.run/'viewer.html').write_text(template)


if __name__=='__main__':main()
