"""Compare identical rays in both installed query APIs inside the current GUI."""
import json,time,math
import numpy as np
from pxr import PhysicsSchemaTools
import live_gui
from lidar import quaternion_matrix

c=live_gui.controller;sensor=c.lidar
legacy=sensor.make_raycaster(dict(sensor.config,backend='physx'))
core=sensor.make_raycaster(dict(sensor.config,backend='physics_core'))
pose=np.asarray(c.body.get_transforms()[0],dtype=float)
rotation=quaternion_matrix(pose[3:])
origin=pose[:3]+rotation@sensor.chassis_offset
origins=np.tile(origin,(1081,1));directions=sensor.chassis_directions@rotation.T
origins=np.vstack((origins,np.tile(pose[:3],(6,1))))
directions=np.vstack((directions,np.vstack((np.eye(3),-np.eye(3)))@rotation.T))

def measure(caster):
    if hasattr(caster,'ranges'):
        return np.asarray(caster.ranges(origins,directions,.06,30.))
    return np.asarray([caster.range(o,d,.06,30.) for o,d in zip(origins,directions)])

reference=measure(legacy);candidate=measure(core)
finite=np.isfinite(reference)&np.isfinite(candidate)
checks={'new_api_used':core.backend=='physics_core',
        'finite_mask_equal':bool(np.array_equal(np.isfinite(reference),np.isfinite(candidate))),
        'infinite_signs_equal':bool(np.array_equal(np.isposinf(reference),np.isposinf(candidate)) and
                                    np.array_equal(np.isneginf(reference),np.isneginf(candidate))),
        'ranges_match':bool(np.any(finite) and np.max(np.abs(reference[finite]-candidate[finite]))<1e-6)}
timings={}
for caster in (legacy,core):
    values=[]
    for _ in range(5):
        start=time.perf_counter();measure(caster);values.append(time.perf_counter()-start)
    timings[caster.backend]=float(np.median(values))
decoded=None
if core.backend=='physics_core':
    found,hit=core.query.raycast_closest(core.vector_type(*origin),core.vector_type(*directions[540]),30.,True)
    if found:decoded={'collision':str(PhysicsSchemaTools.intToSdfPath(hit.collision)),
                      'rigid_body':str(PhysicsSchemaTools.intToSdfPath(hit.rigid_body))}
report={'status':'pass' if all(checks.values()) else 'fail','checks':checks,'rays':len(origins),
        'self_test_rays':6,'maximum_range_difference_m':float(np.max(np.abs(reference[finite]-candidate[finite]))) if finite.any() else None,
        'median_query_wall_s':timings,'decoded_core_hit':decoded}
(c.output_dir/'performance/lidar_backend_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
