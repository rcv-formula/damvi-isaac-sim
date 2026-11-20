"""Open the existing car/map asset as a GUI and optionally attach the adapters."""
import argparse
import sys
from pathlib import Path

parser=argparse.ArgumentParser()
ROOT=Path(__file__).resolve().parents[1]
DEFAULT_STAGE=ROOT.parent/'rc_car_scene_with_0821.usd'
parser.add_argument('--stage',type=Path,help='Optional custom scene USD')
parser.add_argument('--dataset',help='Track geometry and bag-calibrated friction profile to use')
parser.add_argument('--csv',type=Path,help='Dataset raceline used for the rear-axle spawn')
parser.add_argument('--scene-only',action='store_true')
parser.add_argument('--headless',action='store_true',help='Use the same adapters without a desktop or viewport')
args=parser.parse_args()

from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'width':1600,'height':1000,'renderer':'RayTracedLighting',
                   'disable_viewport_updates':args.headless})
try:
    import carb.settings
    import omni.kit.app,omni.usd
    settings=carb.settings.get_settings()
    settings.set('/damvi/headless',args.headless)
    # Do not switch to a worker dispatcher in a GUI with Python physics callbacks.
    settings.set('/physics/physxDispatcher',False)
    settings.set('/persistent/physics/numThreads',0)
    assert omni.usd.get_context().open_stage(str((args.stage or DEFAULT_STAGE).resolve()))
    if args.dataset or (args.headless and not args.scene_only and args.stage is None):
        sys.path.insert(0,str(ROOT))
        from dataset_scene import configure_dataset
        from reset_dataset import csv_pose
        from steering import load_configuration
        dataset=args.dataset or load_configuration(ROOT/'config/vesc_sim.yaml')['_surface']['selected_dataset']
        path=args.csv or ROOT.parent/'maps/details'/dataset/'raceline.csv'
        configure_dataset(omni.usd.get_context().get_stage(),dataset,csv_pose(path,0,0.))
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
        'isaacsim.code_editor.python_server',True)
    if not args.headless:
        from omni.kit.viewport.utility import get_active_viewport
        viewport=get_active_viewport()
        if viewport:
            viewport.camera_path='/World/MapOverview'
    if not args.scene_only:
        install=Path(__file__).with_name('install_gui.py')
        exec(compile(install.read_text(),str(install),'exec'),globals())
    print('[DAMVI_GUI] Existing car/map loaded; localhost Python Server ready',flush=True)
    while app.is_running():
        app.update()
finally:
    app.close()
