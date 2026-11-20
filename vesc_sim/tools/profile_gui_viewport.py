"""Temporarily isolate viewport rendering, with guaranteed restoration."""
import asyncio,json,time
import omni.kit.app
from omni.kit.viewport.utility import get_active_viewport
import live_gui


async def compare():
    c=live_gui.controller;app=omni.kit.app.get_app();viewport=get_active_viewport()
    original=viewport.updates_enabled;results=[]
    try:
        for enabled in (True,False,True):
            viewport.updates_enabled=enabled
            for _ in range(5):await app.next_update_async()
            sim_start=c.elapsed;wall_start=time.perf_counter();frames=0
            while time.perf_counter()-wall_start<4.:
                await app.next_update_async();frames+=1
            wall=time.perf_counter()-wall_start;sim=c.elapsed-sim_start
            results.append({'viewport_updates':enabled,'wall_s':wall,'sim_s':sim,
                            'realtime_factor':sim/wall,'frames':frames})
    finally:
        viewport.updates_enabled=original
    (c.output_dir/'performance/viewport_trials.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))


viewport_profile_task=asyncio.ensure_future(compare())
print('Temporary viewport off/on comparison scheduled')
