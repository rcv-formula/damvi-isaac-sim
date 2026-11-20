def _finish_movie_recording():
    import json
    from pathlib import Path
    import live_gui
    output=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/outputs/dual_view_120s_20261113'
    if '_damvi_movie_contact_monitor' in globals():
        _damvi_movie_contact_monitor.duration_s=0.
    if '_damvi_follow_camera' in globals():
        camera=_damvi_follow_camera
        (output/'camera_trajectory.json').write_text(json.dumps({'samples':camera.samples,'error':camera.error},indent=2)+'\n')
        for layer,visible in camera.overlays:
            layer.visible=visible
    c=live_gui.controller
    c.write_status()
    (output/'final_sim_status.json').write_text((c.output_dir/'status.json').read_text())
    (output/'follow_camera_viewport.json').write_text((c.output_dir/'follow_camera_viewport.json').read_text())
    (output/'follow_camera_config.yaml').write_text((Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/config/recording_camera.yaml').read_text())
    print(json.dumps({'target_erpm':c.target,'controller_error':c.error,'physics_hz':c.config['physics_hz'],'realtime_factor':c.realtime_factor}))
_finish_movie_recording()
