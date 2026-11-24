"""Explicit hypothetical surface-following dry runs; no planning or execution."""
import math
import numpy as np


def surface_pose(kind,center,u,v,stand_off=.005):
    """Return XYZ and tool +Z into material, with a positive air gap.

    u is surface arc length on curved objects; v is vertical displacement.
    Centers/placements and scene fixture dimensions are illustrative only.
    """
    cx,cy,cz=map(float,center)
    if kind=='cylinder':
        angle=u/.04;radius=.04+stand_off
        return [cx-radius*math.cos(angle),cy+radius*math.sin(angle),cz+v],[math.cos(angle),-math.sin(angle),0.]
    if kind=='plane':return [cx-stand_off,cy+u,cz+v],[1.,0.,0.]
    if kind=='quarter_pipe':
        angle=u/.10;radius=.10-stand_off
        return [cx+radius*math.cos(angle),cy+radius*math.sin(angle),cz+v],[math.cos(angle),math.sin(angle),0.]
    raise ValueError('Unknown nominal surface')


def dry_run_loop(kind='cylinder',center=None,span=.0508,stand_off=.005,speed=.003,samples=41):
    """A smooth closed oval, 50.8 mm across and vertically in surface coordinates.

    Forty intermediate intervals do not create stops: this is one timed trace
    with explicit endpoint ramps and an independently planned pen-up approach.
    """
    if kind not in ('cylinder','plane','quarter_pipe'):raise ValueError('Unknown nominal surface')
    if center is None:center={'cylinder':[.4,.1,.22],'plane':[.36,.1,.22],'quarter_pipe':[.25,.1,.22]}[kind]
    if len(center)!=3 or not all(math.isfinite(v) for v in center):raise ValueError('Invalid surface placement')
    if not isinstance(samples,int) or samples<9 or samples>80 or not all(math.isfinite(v) and v>0 for v in (span,stand_off,speed)):
        raise ValueError('Invalid dry-run parameters')
    if kind=='quarter_pipe' and stand_off>=.1:raise ValueError('Stand-off exceeds pipe cavity radius')
    points=[]
    for phase in np.linspace(0.,2*math.pi,samples):
        xyz,direction=surface_pose(kind,center,span/2*math.cos(phase),span/2*math.sin(phase),stand_off)
        points.append(dict(xyz=xyz,direction=direction,contact=False))
    return dict(schema_version=1,name=f'{kind.replace("_"," ").title()}: continuous 2-inch dry run, {speed*1000:g} mm/s',
        frame='base_link',tcp_frame='pointer_tip',orientation_mode='tool_axis',style='spline',
        joint_speed_deg_s=30.,joint_acceleration_deg_s2=180.,tcp_speed_m_s=speed,tcp_acceleration_m_s2=.02,
        plan_pen_up_approach=True,allow_contact_ramps=False,enforce_tcp_speed=True,waypoints=points,
        notes=f'Hypothetical pen-up dry run, not contact. Smooth oval spans {span*1000:g} mm in both surface coordinates. {stand_off*1000:g} mm stand-off, normals preserved. Approach is pen-up; cruise speed is enforced through all interior points. Confirm actual scene and tool before physical use.',
        source=dict(generator='nominal_paths.dry_run_loop',surface=kind,center_m=list(center),surface_span_m=[span,span],
            stand_off_m=stand_off,nominal_radius_m={'cylinder':.04,'plane':0.,'quarter_pipe':.10}[kind],
            scene=f'nominal_{kind}_dry_run.yaml',hypothetical_placement=True,physical_contact_qualified=False))


def surface_measurement(kind,center,position):
    """Analytic normal and positive air gap at a measured/model TCP position."""
    center=np.asarray(center,dtype=float);position=np.asarray(position,dtype=float)
    if center.shape!=(3,) or position.shape!=(3,) or not np.isfinite(np.r_[center,position]).all():
        raise ValueError('Finite XYZ surface center and TCP required')
    if kind=='plane':return np.array([1.,0.,0.]),float(center[0]-position[0])
    if kind=='tabletop':return np.array([0.,0.,-1.]),float(position[2]-center[2])
    if kind not in ('cylinder','quarter_pipe'):raise ValueError('Unknown nominal surface')
    radial=np.r_[position[:2]-center[:2],0.];radius=np.linalg.norm(radial)
    if radius<1e-9:raise ValueError('Surface normal undefined on axis')
    return (-radial/radius,float(radius-.04)) if kind=='cylinder' else (radial/radius,float(.10-radius))


def coverage_strokes(kind):
    """Boundary/height witnesses, not a claim of arbitrary interior coverage.

    The cylinder's proposed test band is -75..+45 degrees at this offset
    placement; full front-180 coverage remains a separate
    unresolved requirement. Plane/pipe use their full nominal widths. All paths
    remain 5 mm in air, 5 mm inside the top/bottom rims, and require strict speed.
    """
    data=dry_run_loop(kind)
    extent={'cylinder':.04*math.pi/3,'plane':.05,'quarter_pipe':.10*math.pi/4}[kind]
    low,high=(-.04*math.pi*5/12,.04*math.pi/4) if kind=='cylinder' else (-extent,extent)
    output=[]
    for name,coordinates in (
        ('lower_horizontal',zip(np.linspace(low,high,31),np.full(31,-.045))),
        ('center_vertical',zip(np.zeros(31),np.linspace(-.045,.045,31))),
        ('upper_horizontal',zip(np.linspace(high,low,31),np.full(31,.045)))):
        import copy
        stroke=copy.deepcopy(data);stroke['waypoints']=[];stroke['name']=f'{kind}: {name}, strict 3 mm/s'
        for u,v in coordinates:
            xyz,direction=surface_pose(kind,data['source']['center_m'],float(u),float(v),.005)
            stroke['waypoints'].append(dict(xyz=xyz,direction=direction,contact=False))
        stroke['source'].update(generator='nominal_paths.coverage_strokes',case=name,
            surface_u_interval_m=[low,high],surface_span_m=[0. if name=='center_vertical' else high-low,.09 if name=='center_vertical' else 0.],
            scene=f'nominal_{kind}_pedestal_dry_run.yaml',supported_area_claim=False)
        stroke['notes']='Proposed pedestal scene only. Boundary/height witness, not arbitrary-path coverage or physical qualification. Strict 3 mm/s with normals preserved and 5 mm air gap.'
        output.append(stroke)
    return output


def fast_cylinder_loop():
    """Three-inch oval on the nominal cylinder at an explicit lower placement."""
    data=dry_run_loop(center=[.4,.1,.15],span=.0762,speed=.007,samples=61)
    data.update(name='Cylinder: continuous 3-inch dry run, 7 mm/s',style='spline_axis',waypoints=[])
    offset=-.04*math.pi/12
    for phase in np.linspace(0.,2*math.pi,61):
        xyz,direction=surface_pose('cylinder',data['source']['center_m'],offset+.0381*math.cos(phase),.0381*math.sin(phase),.005)
        data['waypoints'].append(dict(xyz=xyz,direction=direction,contact=False))
    data['source'].update(generator='nominal_paths.fast_cylinder_loop',scene='fast_cylinder_pedestal_dry_run.yaml',surface_u_center_m=offset)
    data['notes']+=' Explicit alternative placement: cylinder center Z=150 mm, surface oval centered at -15 degrees. Smooth position and unit pen direction interpolation; nominal cylinder dimensions remain unchanged.'
    return data


def bounded_cylinder_loop():
    """Explicit 4–18 mm/s air-loop band; fixed-speed examples stay unchanged."""
    data=fast_cylinder_loop()
    data.update(name='Cylinder: 3-inch dry run, TCP range 4–18 mm/s',tcp_min_speed_m_s=.004,tcp_speed_m_s=.018,tcp_acceleration_m_s2=.08)
    data['source']=dict(data['source'],timing_mode='bounded_tcp_speed',generator='nominal_paths.bounded_cylinder_loop')
    data['notes']+=' Explicit smooth interior speed range 4–18 mm/s, with endpoint ramps outside the band. Joint caps remain 30 deg/s and 180 deg/s². This does not repair unqualified physical tracking.'
    return data


def paper_plane_loop(foot_front=.06463529232458132,table_z=-.0024):
    """Owner's 100 mm circle, 5.5 inches beyond the current CAD front feet.

    Surface normal is +Z; the pen's +Z points down into the surface. This is
    an explicit 5 mm air trace, not qualified contact drawing. Current saved
    wrist limits have not yielded this tabletop placement; never bypass them.
    """
    if not all(math.isfinite(v) for v in (foot_front,table_z)):raise ValueError('Finite paper placement required')
    center=[foot_front+.1397,0.,table_z]
    points=[dict(xyz=[center[0]+.05*math.cos(a),.05*math.sin(a),table_z+.005],direction=[0.,0.,-1.],contact=False)
            for a in np.linspace(0,2*math.pi,65)]
    return dict(schema_version=1,name='Tabletop: 100 mm circle, TCP range 4–18 mm/s',frame='base_link',tcp_frame='pointer_tip',
        orientation_mode='tool_axis',style='spline_axis',waypoints=points,joint_speed_deg_s=30.,joint_acceleration_deg_s2=180.,
        tcp_min_speed_m_s=.004,tcp_speed_m_s=.018,tcp_acceleration_m_s2=.08,plan_pen_up_approach=True,
        allow_contact_ramps=False,enforce_tcp_speed=True,
        notes='Requested horizontal paper placement, not a passing saved-limit trajectory or contact qualification. '
              'Surface normal up (+Z), pen axis down (-Z), 5 mm air gap. Center is 139.7 mm beyond current CAD front feet; '
              'letter paper has its 11-inch length along +X and 8.5-inch width along Y. '
              'Current saved wrist limits have not admitted this placement. Confirm actual feet/table/TCP and review feasibility before physical use.',
        source=dict(generator='nominal_paths.paper_plane_loop',surface='tabletop',shape='circle',center_m=center,
            surface_span_m=[.1,.1],stand_off_m=.005,normal_outward=[0.,0.,1.],front_foot_x_m=foot_front,
            center_from_front_foot_m=.1397,paper_size_xy_m=[.2794,.2159],
            scene='tabletop_paper_dry_run.yaml',hypothetical_placement=True,physical_contact_qualified=False,
            saved_limits_qualified=False))


def paper_square_strokes(foot_front=.06463529232458132,table_z=-.0024):
    """Four exact straight sides, each followed by a separate 10 mm lift.

    Separate plan/preview/execute operations retain the existing gateway. No
    square corner is rounded or traversed with an impossible positive speed.
    The next side's checked pen-up approach returns from the lift to its start.
    """
    import copy
    base=paper_plane_loop(foot_front,table_z);center=np.array(base['source']['center_m'])
    corners=np.array([[-.05,-.05],[.05,-.05],[.05,.05],[-.05,.05],[-.05,-.05]])
    output=[]
    for i,(first,last) in enumerate(zip(corners,corners[1:]),1):
        side=copy.deepcopy(base);side.update(name=f'Tabletop square {i}/4: straight 100 mm side, TCP range 4–18 mm/s',style='polyline')
        side['waypoints']=[dict(xyz=[center[0]+float(x),float(y),table_z+.005],direction=[0.,0.,-1.],contact=False)
                           for x,y in np.linspace(first,last,11)]
        side['source'].update(generator='nominal_paths.paper_square_strokes',shape='square',side_index=i,operation='side')
        side['notes']+=' Exact square: execute this straight side, then its matching lift, before loading the next side. No corner blending.'
        lift=copy.deepcopy(side);end=copy.deepcopy(side['waypoints'][-1]);above=copy.deepcopy(end);above['xyz'][2]+=.010
        lift.update(name=f'Tabletop square {i}/4: 10 mm pen-up lift',waypoints=[end,above],tcp_min_speed_m_s=0.,plan_pen_up_approach=False)
        lift['source'].update(operation='lift',lift_m=.010)
        lift['notes']='Separate pen-up retract from the preceding side endpoint, with the pen axis down. Current saved-limit tabletop placement is unqualified. Plan and execute only after the matching side; then load the next side with its checked approach. Lift ramps start/end at rest; the 4–18 mm/s band applies to the drawing sides.'
        output.extend([side,lift])
    return output
