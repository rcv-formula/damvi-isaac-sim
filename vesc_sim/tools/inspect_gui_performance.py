import json
import carb.settings
import omni.kit.app
import omni.physx
import live_gui
from pxr import UsdPhysics,PhysxSchema

c=live_gui.controller
s=carb.settings.get_settings()
keys=['/persistent/physics/numThreads','/defaults/persistent/physics/numThreads',
      '/physics/physxDispatcher','/plugins/carb.tasking.plugin/threadCount',
      '/physics/updateToUsd','/physics/updateVelocitiesToUsd','/physics/updateTransformationsToUsd',
      '/persistent/physics/updateToUsd','/app/renderer/skipWhileMinimized']
manager=omni.kit.app.get_app().get_extension_manager()
scenes=[]
for prim in c.stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        api=PhysxSchema.PhysxSceneAPI(prim)
        scenes.append({'path':str(prim.GetPath()),'gpu_dynamics':api.GetEnableGPUDynamicsAttr().Get(),
                       'broadphase':api.GetBroadphaseTypeAttr().Get(),
                       'solver':api.GetSolverTypeAttr().Get()})
print(json.dumps({'settings':{k:s.get(k) for k in keys},'scenes':scenes,
    'fabric_enabled':manager.is_extension_enabled('omni.physx.fabric'),
    'physx_interface_threads_api':hasattr(omni.physx.get_physx_interface(),'set_thread_count')},indent=2))
