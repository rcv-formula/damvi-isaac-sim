"""Run in the existing GUI via isaac_live_client.py; returns immediately."""
import sys,asyncio,importlib
import importlib.util
from pathlib import Path
folder='/home/shin/Desktop/RC_Car_Isaac/vesc_sim'
if folder not in sys.path:sys.path.insert(0,folder)
importlib.invalidate_caches()
# Kit's fast importer can cache a directory before newly authored modules exist.
# Load these concrete project files explicitly in a long-running GUI process.
for module_name in ('steering','steering_scene','wheel_collision','tire_model','tire_physics','ros_bridge','tf_config','lidar','imu'):
    source=Path(folder)/(module_name+'.py')
    if module_name not in sys.modules:
        spec=importlib.util.spec_from_file_location(module_name,source)
        module=importlib.util.module_from_spec(spec);sys.modules[module_name]=module
        spec.loader.exec_module(module)
if 'live_gui' in sys.modules:
    old=getattr(sys.modules['live_gui'],'controller',None)
    if old:
        old.close()
        window=getattr(old,'window',None)
        if window:window.visible=False;window.destroy()
    module=sys.modules['live_gui'];source=Path(module.__file__)
    exec(compile(source.read_text(),str(source),'exec'),module.__dict__)
import live_gui
# Reload project adapters as source when adding a feature to a running GUI.
for module_name in ('ros_bridge','steering','steering_scene','wheel_collision','tire_model','tire_physics','tf_config','lidar','imu'):
    if module_name in sys.modules:
        module=sys.modules[module_name];source=Path(module.__file__)
        exec(compile(source.read_text(),str(source),'exec'),module.__dict__)
# Bind the freshly loaded classes in live_gui as well.
live_gui.SteeringController=sys.modules['steering'].SteeringController
live_gui.load_configuration=sys.modules['steering'].load_configuration
live_gui.prepare_front_steering=sys.modules['steering_scene'].prepare_front_steering
live_gui.prepare_wheel_collisions=sys.modules['wheel_collision'].prepare_wheel_collisions
live_gui.apply_wheel_angular_limits=sys.modules['wheel_collision'].apply_wheel_angular_limits
live_gui.prepare_tire_physics=sys.modules['tire_physics'].prepare_tire_physics
live_gui.TirePhysics=sys.modules['tire_physics'].TirePhysics
live_gui.LidarSensor=sys.modules['lidar'].LidarSensor
live_gui.ImuSensor=sys.modules['imu'].ImuSensor
live_gui.FrameTree=sys.modules['tf_config'].FrameTree
live_gui.StaticFrames=sys.modules['tf_config'].StaticFrames
vesc_gui_task=asyncio.ensure_future(live_gui.install())
print('Existing-GUI controller installation scheduled')
