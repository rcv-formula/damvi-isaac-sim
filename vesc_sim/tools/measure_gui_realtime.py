"""Measure wall/simulation rates with active sensors, without altering settings."""
import asyncio,json,time
import carb.settings
import omni.kit.app
import live_gui
from omni.kit.loop import _loop


async def measure():
    c=live_gui.controller;app=omni.kit.app.get_app()
    for _ in range(120):
        if c.error:raise RuntimeError(c.error)
        if c.art is not None and c.imu is not None and c.elapsed>=1.:break
        await app.next_update_async()
    start=time.perf_counter();sim=c.elapsed;frames=0
    imu=c.imu.published_count;scans=c.lidar.published_count;previous=start;previous_sim=sim
    windows=[]
    while time.perf_counter()-start<10.:
        await app.next_update_async();frames+=1
        now=time.perf_counter()
        if now-previous>=1.:
            windows.append({'wall_s':now-previous,'sim_s':c.elapsed-previous_sim,
                            'realtime_factor':(c.elapsed-previous_sim)/(now-previous)})
            previous=now;previous_sim=c.elapsed
    wall=time.perf_counter()-start;elapsed=c.elapsed-sim
    s=carb.settings.get_settings()
    report={'wall_s':wall,'sim_s':elapsed,'realtime_factor':elapsed/wall,
            'frames':frames,'average_fps':frames/wall,'simulation_s_per_frame':elapsed/frames,
            'imu_messages':c.imu.published_count-imu,'scans':c.lidar.published_count-scans,
            'physics_hz':c.config['physics_hz'],'render_hz':c.config['_runtime']['render_hz'],
            'stage_time_codes_per_second':c.stage.GetTimeCodesPerSecond(),
            'timeline_target_framerate':c.timeline.get_target_framerate(),
            'rate_limit_enabled':s.get('/app/runLoops/main/rateLimitEnabled'),
            'rate_limit_frequency':s.get('/app/runLoops/main/rateLimitFrequency'),
            'fixed_timestep':s.get('/app/player/useFixedTimeStepping'),
            'manual_loop':_loop.acquire_loop_interface().get_manual_mode(),
            'error':c.error,'windows':windows}
    (c.output_dir/'performance/realtime_check.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


realtime_measure_task=asyncio.ensure_future(measure())
print('Ten-second simulation/wall-time measurement scheduled')
