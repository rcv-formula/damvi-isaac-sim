"""Bounded, reversible measurements inside the existing Isaac GUI only."""
import asyncio
import cProfile
import json
import pstats
import time
from pathlib import Path

import carb.settings
import omni.kit.app
import omni.physx
import live_gui


async def profile():
    controller = live_gui.controller
    app = omni.kit.app.get_app()
    directory = controller.output_dir/'performance'
    directory.mkdir(exist_ok=True)
    measurements = []
    methods = [(controller.native, 'step', 'motor'), (controller.steering, 'step', 'steering'),
               (controller.lidar, 'update', 'lidar'), (controller.imu, 'update', 'imu'),
               (controller.bridge, 'publish', 'ros_vesc_clock'),
               (controller.bridge, 'spin', 'ros_spin'), (controller, 'write_status', 'status_files')]
    originals = [(obj, name, getattr(obj, name), label) for obj,name,label in methods]
    sample = {'steps': 0, 'sim_time': 0., 'frames': 0}

    def tick(dt):
        sample['steps'] += 1
        sample['sim_time'] += float(dt)

    def frame(event):
        sample['frames'] += 1

    tick_subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(tick, False, 100)
    frame_subscription = app.get_update_event_stream().create_subscription_to_pop(frame, name='damvi_bounded_profile')
    profiler = cProfile.Profile()
    try:
        for label, skipped in [('all_enabled', set()), ('without_lidar', {'lidar'}),
                               ('without_imu', {'imu'}), ('without_both_sensors', {'lidar','imu'}),
                               ('without_sensors_or_status', {'lidar','imu','status_files'})]:
            totals = {name: {'calls': 0, 'wall_s': 0.} for _,_,_,name in originals}
            for obj,name,original,component in originals:
                if component in skipped:
                    setattr(obj, name, lambda *args, **kwargs: 0)
                else:
                    def measured(*args, _method=original, _label=component, **kwargs):
                        start = time.perf_counter()
                        try:
                            return _method(*args, **kwargs)
                        finally:
                            totals[_label]['calls'] += 1
                            totals[_label]['wall_s'] += time.perf_counter()-start
                    setattr(obj, name, measured)
            # Let a partial rolling scan be discarded by reset after each trial.
            controller.lidar.reset(); controller.imu.reset()
            for _ in range(3):
                await app.next_update_async()
            begin = dict(sample)
            started = time.perf_counter()
            while time.perf_counter()-started < 4.:
                await app.next_update_async()
            wall = time.perf_counter()-started
            sim = sample['sim_time']-begin['sim_time']
            measurements.append({'trial': label, 'wall_s': wall, 'sim_s': sim,
                                 'realtime_factor': sim/wall,
                                 'physics_steps': sample['steps']-begin['steps'],
                                 'render_frames': sample['frames']-begin['frames'],
                                 'components': totals})
            (directory/'profile_progress.json').write_text(json.dumps(measurements, indent=2))
        for obj,name,original,_ in originals:
            setattr(obj, name, original)
        controller.lidar.reset(); controller.imu.reset()
        profiler.enable()
        started = time.perf_counter()
        while time.perf_counter()-started < 3.:
            await app.next_update_async()
        profiler.disable()
        profiler.dump_stats(str(directory/'gui.prof'))
        stats = pstats.Stats(profiler)
        top = []
        for (filename,line,name), (primitive,calls,own,cumulative,callers) in sorted(
                stats.stats.items(), key=lambda entry: entry[1][3], reverse=True)[:65]:
            top.append(dict(file=filename,line=line,name=name,calls=calls,own_s=own,cumulative_s=cumulative))
        settings=carb.settings.get_settings()
        keys=['/app/runLoops/main/rateLimitEnabled','/app/runLoops/main/rateLimitFrequency',
              '/app/player/useFixedTimeStepping','/persistent/simulation/minFrameRate',
              '/physics/minFrameRate','/physics/sceneMultiGPUMode','/app/asyncRendering']
        report={'trials':measurements,'profile_wall_s':time.perf_counter()-started,
                'profile_top_cumulative':top,'settings':{key:settings.get(key) for key in keys},
                'config_physics_hz':controller.config['physics_hz'],
                'timeline_target_framerate':controller.timeline.get_target_framerate()}
        (directory/'profile.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        profiler.disable()
        for obj,name,original,_ in originals:
            setattr(obj, name, original)
        controller.lidar.reset(); controller.imu.reset()
        tick_subscription = None
        frame_subscription = None
        controller.write_status()


performance_profile_task = asyncio.ensure_future(profile())
print('Profiling current GUI in bounded trials; sensor streams briefly pause and then resume')
