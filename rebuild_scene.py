"""Rebuild the final CAD vehicle with the saved configuration using Isaac USD Python.

Example: ./run_usd_python.sh /home/shin/Desktop/RC_Car_Isaac/rebuild_scene.py
Run before opening the generated output stage. Source OBJ/MTL are only needed
for rebuilding, not for opening the self-contained final USD.
"""
import json
from pathlib import Path
from pxr import Gf, UsdGeom

package = Path(__file__).resolve().parent
config = json.loads((package/'build_config.json').read_text(encoding='utf-8'))
namespace = {'__name__': 'rc_builder'}
source = package/'build_rc_scene.py'
exec(compile(source.read_text(encoding='utf-8-sig'), str(source), 'exec'), namespace)
output = package/config['output_usd']
stage, report = namespace['build_scene'](
    package/config['source_obj'], output,
    visual_transform=config['visual_transform'],
    spawn_lift=config['spawn_lift'],
    collision_proxies=config['collision_proxies'],
    tire_visual_segments=config.get('tire_visual_segments', 256))
camera = UsdGeom.Camera(stage.GetPrimAtPath('/World/Camera'))
view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*config['camera_eye_world_m']),
    Gf.Vec3d(*config['camera_target_world_m']), Gf.Vec3d(0,0,1))
camera.GetOrderedXformOps()[0].Set(view.GetInverse())
vehicle = stage.GetPrimAtPath('/World/RC_Car')
vehicle.SetDisplayName('RC Car - Fusion platform-nuc-one-plate v6')
vehicle.SetCustomDataByKey('source_cad_document', 'platform-nuc-one-plate v6')
vehicle.SetCustomDataByKey('actual_fusion_mesh_imported', True)
report['actual_fusion_mesh_imported'] = True
report['alignment_notes'] = config['alignment_notes']
report['camera_eye_world_m'] = config['camera_eye_world_m']
report['camera_target_world_m'] = config['camera_target_world_m']
metadata = dict(stage.GetRootLayer().customLayerData)
metadata['rc_car_build_report_json'] = json.dumps(report, ensure_ascii=True)
stage.GetRootLayer().customLayerData = metadata
stage.GetRootLayer().Save()
output.with_suffix('.build.json').write_text(json.dumps(report, indent=2, ensure_ascii=True), encoding='utf-8')
print(json.dumps({key:report[key] for key in (
    'output_usd','actual_fusion_mesh_imported','visual_dimensions_m','visual_meshes','visual_faces',
    'measured_initial_mass_kg','measured_initial_com_world_m','measured_initial_yaw_inertia_kg_m2')}, indent=2))
