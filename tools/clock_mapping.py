"""Offline hardware-counter to host-clock mapping, retaining all raw observations.

Host read midpoints are observations with finite brackets, not sample timestamps.
Each fifth usable observation is withheld from fitting. Local linear fits form a
continuous piecewise-linear clock map, so clock drift is not folded into IMU dt.
This does not estimate sensor/filter or camera exposure delays.
"""
import numpy as np


def fit_clock(anchors):
    x=np.array([a['sensor_ticks_u32'] for a in anchors],dtype=float)
    before=np.array([a['host_before_boot_ns'] for a in anchors],dtype=float)
    after=np.array([a['host_after_boot_ns'] for a in anchors],dtype=float)
    if len(x)<20 or np.any(np.diff(x)<=0):
        raise ValueError('Need >=20 monotonic anchors; counter reset/rollover requires segmentation')
    widths=after-before
    if np.any(widths<=0) or np.any(np.diff(before)<=0):raise ValueError('Invalid host clock brackets')
    y=(before+after)/2
    # Fixed bracket budget avoids discarding an entire bus-load interval merely
    # because another part of the run had faster reads. At most +/-1 ms bracket.
    usable=widths<=2_000_000
    test=usable & (np.arange(len(x))%5==0)
    train=usable & ~test
    knots=np.unique(np.r_[np.arange(x[0],x[-1],40000.0),x[-1]])
    predicted=[]
    for knot in knots:
        # Keep the same 80000-tick aperture at the boundaries as in the
        # interior. Shift it inward rather than silently halving its width.
        # Bracket quality, minimum observations and withheld checks stay intact.
        low=max(x[0],min(knot-40000,x[-1]-80000))
        high=min(x[-1],low+80000)
        mask=train & (x>=low) & (x<=high)
        if mask.sum()<5:raise ValueError('Insufficient local clock observations; reject rather than invent mapping')
        local_x=x[mask]-knot;local_y=y[mask]-y[0]
        coeff=np.polyfit(local_x,local_y,1)
        predicted.append(y[0]+coeff[1])
    predicted=np.array(predicted)
    scales=np.diff(predicted)/np.diff(knots)
    if np.any((scales<20000)|(scales>30000)):raise ValueError('Implausible/discontinuous clock map')
    def map_time(ticks):
        values=np.asarray(ticks,dtype=float)
        index=np.clip(np.searchsorted(knots,values,side='right')-1,0,len(knots)-2)
        return predicted[index]+(values-knots[index])*scales[index]
    residual=y-map_time(x)
    global_coeff=np.polyfit(x[train]-x[0],y[train]-y[0],1)
    global_residual=y-(y[0]+np.polyval(global_coeff,x-x[0]))
    info={'model':'piecewise-linear host mapping from overlapping local affine fits',
          'nominal_knot_spacing_ticks':40000,'local_half_window_ticks':40000,
          'boundary_window':'80000-tick window shifted inward at recording endpoints',
          'anchor_read_width_cutoff_ns':2_000_000,
          'anchors_total':len(x),'anchors_fitted':int(train.sum()),'anchors_withheld':int(test.sum()),
          'knots_sensor_ticks':knots.tolist(),'knots_host_boot_ns':predicted.tolist(),
          'sensor_ns_per_tick_min':float(scales.min()),'sensor_ns_per_tick_max':float(scales.max()),
          'withheld_residual_p95_us':float(np.percentile(abs(residual[test]),95)/1000),
          'withheld_residual_max_us':float(np.max(abs(residual[test]))/1000),
          'global_affine_diagnostic':{'ns_per_tick':float(global_coeff[0]),
              'withheld_residual_p95_us':float(np.percentile(abs(global_residual[test]),95)/1000)},
          'uncertainty_note':'Withheld residual checks clock-map repeatability, not absolute synchronization. Raw read brackets, register latching, internal sensor/filter delay, camera readout and calibrated offset remain relevant.'}
    return map_time,info,residual,widths,usable
