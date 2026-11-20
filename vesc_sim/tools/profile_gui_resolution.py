"""Measure viewport resolution costs while keeping physics and sensors fixed."""
import asyncio,json,time
import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport
import live_gui


async def compare():
    c=live_gui.controller;app=omni.kit.app.get_app();viewport=get_active_viewport()
    original=tuple(viewport.get_texture_resolution());results=[]
    try:
        for resolution in ((1600,1000),(1280,720),(1024,640)):
            viewport.set_texture_resolution(resolution)
            for _ in range(10):await app.next_update_async()
            sim_start=c.elapsed;wall_start=time.perf_counter();frames=0
            while time.perf_counter()-wall_start<5.:
                await app.next_update_async();frames+=1
            wall=time.perf_counter()-wall_start;sim=c.elapsed-sim_start
            results.append({'viewport_resolution':list(viewport.get_texture_resolution()),'wall_s':wall,
                            'sim_s':sim,'realtime_factor':sim/wall,'frames':frames})
    finally:
        viewport.set_texture_resolution(original)
    (c.output_dir/'performance/resolution_trials.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))


resolution_profile_task=asyncio.ensure_future(compare())
print('Viewport resolution comparison scheduled; sensor/physics rates stay fixed')
