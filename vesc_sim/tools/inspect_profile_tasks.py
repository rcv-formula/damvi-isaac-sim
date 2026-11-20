import json
from omni.kit.viewport.utility import get_active_viewport
viewport=get_active_viewport()
print('Viewport resolution attributes',[(n,str(getattr(viewport,n))) for n in dir(viewport) if 'resolution' in n])
for name in ('viewport_profile_task','resolution_profile_task','performance_profile_task'):
    task=globals().get(name)
    if task is not None:
        print(name,'done',task.done(),'exception',repr(task.exception()) if task.done() else None)
