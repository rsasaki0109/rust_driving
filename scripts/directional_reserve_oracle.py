"""Independent trigonometric support and accelerated-path clearance checks."""
import math
from check_hazards import prediction_radius_at

def directional_reserve(velocity, error, tangent):
    speed=math.hypot(*velocity);length=math.hypot(*tangent)
    if not all(math.isfinite(x) for x in (*velocity,*tangent,error)) or error<0.:
        raise ValueError('invalid direction context')
    if speed==0. and error==0.: return 0.
    if length<=1e-6: return max(.4*min(speed+error,1.),.7)
    unit=(tangent[0]/length,tangent[1]/length)
    along=velocity[0]*unit[0]+velocity[1]*unit[1]
    across=velocity[0]*unit[1]-velocity[1]*unit[0]
    if speed<=error:
        angular=1.
    else:
        center=math.atan2(across,along);width=math.asin(error/speed)
        lo,hi=center-width,center+width
        candidates=[abs(math.sin(lo)),abs(math.sin(hi))]
        for k in range(-3,4):
            peak=math.pi/2+k*math.pi
            if lo<=peak<=hi: candidates.append(1.)
        angular=max(candidates)
    return max(.4*min(abs(across)+error,1.),.7*angular)

def certify(config, ticks):
    if not config.get('replan_checked_approaches'): return dict(passed=True,active=False,certified_paths=0,interval_bounds=0)
    bounds=config['forecast_envelope'];tested=paths=0;odom=None
    for tick in ticks:
        inp,out=tick['input'],tick['expected']
        acquired=inp.get('odometry')
        if acquired and acquired['stamp']<=out['time'] and (odom is None or acquired['stamp']>odom['stamp']):
            odom=acquired
        if out['health'] or out['emergency']:continue
        points=out['trajectory']['points']
        if len(points)<2:raise ValueError('healthy selected path lacks a complete interval')
        tracks={t['id']:t for t in out['tracks']}
        if len(tracks)!=len(out['tracks']):raise ValueError('duplicate world-track identity')
        yaw=out['estimate']['pose']['yaw']
        if config.get('rear_axle_offset_m') is not None and odom and odom['speed']>.1:
            yaw+=math.atan2(config['rear_axle_offset_m']*odom['yaw_rate'],odom['speed'])
        held=(math.cos(yaw),math.sin(yaw))
        selected=(out.get('planning_diagnostics') or {}).get('selected_candidate')
        floor=1. if selected is not None and selected>=3 else 0.
        for a,b in zip(points,points[1:]):
            duration=b['time']-a['time']
            start=(a['position']['x'],a['position']['y']);end=(b['position']['x'],b['position']['y'])
            distance=math.dist(start,end)
            if not duration>0.:raise ValueError('nonpositive selected interval')
            tangent=tuple((end[k]-start[k])/distance for k in range(2)) if distance>1e-6 else held
            acceleration=(b['speed']-a['speed'])/duration
            def ego_at(t):
                elapsed=t-a['time'];fraction=(a['speed']*elapsed+acceleration*elapsed*elapsed/2)/distance if distance>1e-12 else 0.
                return tuple(start[k]+(end[k]-start[k])*fraction for k in range(2))
            for obj in out['predictions']:
                track=tracks.get(obj['id'])
                if track is None:raise ValueError('selected path lacks owned direction context')
                age=out['time']-track['last_seen']
                if not age>=0.:raise ValueError('future track')
                velocity=(track['velocity']['x'],track['velocity']['y'])
                positions=obj['positions'];dt=obj['dt']
                knots=sorted({a['time'],b['time'],*[i*dt for i in range(len(positions)) if a['time']<i*dt<b['time']]})
                for lower,upper in zip(knots,knots[1:]):
                    middle=(lower+upper)/2;index=min(int(middle/dt),len(positions)-1);nxt=min(index+1,len(positions)-1)
                    nominal=tuple((positions[nxt][k]-positions[index][k])/dt for k in ('x','y'))
                    nominal_speed=math.hypot(*nominal)
                    transverse=abs(nominal[0]*tangent[1]-nominal[1]*tangent[0])
                    legacy=max(.4*min(transverse,1.),.7*transverse/nominal_speed) if nominal_speed else 0.
                    supplied=directional_reserve(velocity,
                        bounds['velocity_error_m_s']+bounds['acceleration_error_m_s2']*(age+upper),tangent)
                    margin=max(floor,.3+.06*min(upper,5.)+max(legacy,supplied))
                    radius_slope=abs(prediction_radius_at(obj,upper)-prediction_radius_at(obj,lower))/(upper-lower)
                    def gap(t):
                        fraction=(t-index*dt)/dt if index!=nxt else 0.
                        center=tuple(positions[index][k]+(positions[nxt][k]-positions[index][k])*fraction for k in ('x','y'))
                        return math.dist(ego_at(t),center)-config['vehicle']['radius']-prediction_radius_at(obj,t)-margin
                    stack=[(lower,upper,0)]
                    while stack:
                        lo,hi,depth=stack.pop();center=(lo+hi)/2;value=gap(center);tested+=1
                        maximum_speed=max(a['speed']+acceleration*(lo-a['time']),a['speed']+acceleration*(hi-a['time']))
                        bound=value-(maximum_speed+nominal_speed+radius_slope)*(hi-lo)/2
                        if not math.isfinite(value) or value<-1e-8:
                            raise ValueError('selected path contacts uncertainty-supported directional reserve')
                        if bound>1e-9:continue
                        if depth>=24:raise ValueError('directional path clearance unresolved')
                        stack.extend([(lo,center,depth+1),(center,hi,depth+1)])
        paths+=1
    return dict(passed=True,active=True,certified_paths=paths,interval_bounds=tested,
        actor_truth_used=False,uncertainty_calibrated=False)
