"""Arrival punctuality and delayed arrivals during each planned button dwell."""
import numpy as np

from .buttons import button_target


def evaluate_arrivals(arrivals, times, states, actual_tip, timestep, dwell, clearance=None):
    """Evaluate full-rate samples, from scheduled arrival through its dwell.

    The first pose is an initial condition, not a destination. A late hit may
    not borrow samples from the next leg or a later visit to the same button.
    Arrival is the first qualifying sample, not a sustained-settling test.
    `reached` retains its original meaning: the criterion holds ON TIME.
    """
    times=np.asarray(times)
    states=np.asarray(states)
    actual_tip=np.asarray(actual_tip)
    safe=np.ones(len(times),dtype=bool) if clearance is None else np.asarray(clearance)>=0
    dwell_steps=int(np.ceil(dwell/timestep))
    visits=[]
    for order,arrival in enumerate(arrivals):
        idx=int(arrival['index'])
        end=idx+(dwell_steps if order else 0)
        if order+1<len(arrivals):
            end=min(end,int(arrivals[order+1]['index'])-1)
        last=min(end,len(times)-1)
        position=np.asarray(button_target(arrival['button'])['position_m'])
        observed=idx<=last
        errors=np.linalg.norm(actual_tip[idx:last+1]-position,axis=1) if observed else np.array([])
        speeds=np.max(np.abs(states[idx:last+1,6:]),axis=1) if observed else np.array([])
        hits=np.flatnonzero((errors<.01)&(speeds<.05)&safe[idx:last+1]) if observed else []
        hit=int(hits[0]) if len(hits) else None
        status='missed' if hit is None else ('on_time' if hit==0 else 'delayed')
        reason=None if hit is not None else (
            'interrupted_before_arrival' if not observed else
            'interrupted_during_window' if last<end or not safe[last] else 'criteria_not_met')
        visits.append(dict(button=arrival['button'],index=idx,time_s=float(idx*timestep),
            tip_error_m=float(errors[0]) if observed else None,
            speed_rad_s=float(speeds[0]) if observed else None,
            reached=status=='on_time',status=status,reached_within_window=hit is not None,
            actual_arrival_time_s=float(times[idx+hit]) if hit is not None else None,
            delay_s=float(hit*timestep) if hit is not None else None,
            arrival_tip_error_m=float(errors[hit]) if hit is not None else None,
            arrival_speed_rad_s=float(speeds[hit]) if hit is not None else None,
            window_end_time_s=float(end*timestep),
            observation_end_time_s=float(times[last]) if observed else None,reason=reason))
    return visits


def arrival_counts(visits):
    destinations=visits[1:]
    counts={status:sum(v['status']==status for v in destinations)
            for status in ('on_time','delayed','missed')}
    return dict(counts,total=len(destinations),reached=counts['on_time']+counts['delayed'])
