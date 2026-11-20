import json
import omni.usd,omni.timeline,omni.kit.app
from pxr import UsdGeom,UsdPhysics
stage=omni.usd.get_context().get_stage()
timeline=omni.timeline.get_timeline_interface()
print(json.dumps({'stage':stage.GetRootLayer().identifier,'playing':timeline.is_playing(),
    'time':timeline.get_current_time(),'end_time':timeline.get_end_time(),
    'session':stage.GetSessionLayer().identifier,
    'chassis_world_position':list(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/RC_Car/Chassis')).ExtractTranslation()),
    'scenes':[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]}))
