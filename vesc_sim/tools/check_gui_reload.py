import asyncio,json
from pathlib import Path
import omni.kit.app,omni.ui
import live_gui
from omni.kit.loop import _loop

async def check():
    c=live_gui.controller
    previous=c.native
    previous_steering=c.steering
    previous_lidar_publisher=c.lidar.publisher
    previous_imu_publisher=c.imu.publisher
    previous_tf_publisher=c.static_frames.publisher
    before=c.config_load_count
    c.timeline.stop()
    for _ in range(4):await omni.kit.app.get_app().next_update_async()
    c.timeline.play()
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()
        if c.art is not None and c.config_load_count>before:break
    window=omni.ui.Workspace.get_window('DAMVI VESC - forward / reverse')
    checks={'reread_on_play':c.config_load_count>before,
        'controller_recreated':c.native is not previous,
        'steering_recreated':c.steering is not previous_steering,
        'lidar_publisher_recreated':c.lidar.publisher is not previous_lidar_publisher,
        'lidar_config_loaded':c.lidar.config['scan_rate_hz']==40. and c.lidar.config['sample_count']==1081,
        'imu_publisher_recreated':c.imu.publisher is not previous_imu_publisher,
        'tf_publisher_recreated':c.static_frames.publisher is not previous_tf_publisher,
        'shared_tf_loaded':c.lidar.frame_tree is c.frame_tree and c.imu.frame_tree is c.frame_tree,
        'physics_200hz':c.config['physics_hz']==200,
        'imu_1khz_interpolated':c.imu.config['sample_rate_hz']==1000. and c.imu.config['sampling_mode']=='interpolated_physics_state',
        'wall_clock_loop':not _loop.acquire_loop_interface().get_manual_mode(),
        'gain_loaded':c.config['speed_to_erpm_gain']==1538.,
        'no_custom_ui':window is None or not window.visible,
        'ros_connected':c.bridge is not None,
        'no_error':c.error is None,'live_articulation':c.art is not None}
    out={'status':'pass' if all(checks.values()) else 'fail','checks':checks,'config':str(c.config_path)}
    (c.output_dir/'reload_check.json').write_text(json.dumps(out,indent=2))
    c.write_status()
    print(json.dumps(out))

reload_check_task=asyncio.ensure_future(check())
print('Checking YAML reload and custom UI removal in existing GUI')
