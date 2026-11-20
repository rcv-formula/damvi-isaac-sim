"""Run through isaac_live_client.py in the existing GUI session.

Only creates/updates a session-layer visual camera and selects it in the existing
viewport. Physics, commands, scene assets and localization are untouched.
"""


def _install_damvi_recording_camera():
    import json
    import math
    import time
    from pathlib import Path
    import yaml
    import omni.kit.app
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport_and_window
    from pxr import Gf, Usd, UsdGeom
    import live_gui

    config_file = Path.home() / 'Desktop/RC_Car_Isaac/vesc_sim/config/recording_camera.yaml'
    cfg = yaml.safe_load(config_file.read_text())
    controller = live_gui.controller
    stage = omni.usd.get_context().get_stage()
    viewport, window = get_active_viewport_and_window()
    if controller.body is None or viewport is None:
        raise RuntimeError('Existing initialized car and GUI viewport are required')

    class FollowCamera:
        def __init__(self):
            self.previous_camera = str(viewport.camera_path)
            self.previous_resolution = tuple(viewport.resolution)
            self.output = controller.output_dir / 'follow_camera_status.json'
            self.samples = []
            self.last_write = 0.
            self.error = None
            self.subscription = None
            self.overlays = []
            for name, category in (('Axis', 'guide'), ('Menubar', 'menubar')):
                layer = window._find_viewport_layer(name, category)
                if layer is not None:
                    self.overlays.append((layer, layer.visible))
                    layer.visible = False
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                self.camera = UsdGeom.Camera.Define(stage, '/World/RecordingFollowCamera')
                self.camera.CreateFocalLengthAttr(float(cfg['focal_length_mm']))
                self.camera.CreateHorizontalApertureAttr(36.)
                self.camera.CreateVerticalApertureAttr(20.25)
                self.camera.CreateClippingRangeAttr(Gf.Vec2f(0.02, 1000.))
                self.camera.CreateFocusDistanceAttr(2.4)
                self.camera.CreateFStopAttr(0.)
                self.xform = UsdGeom.Xformable(self.camera.GetPrim()).MakeMatrixXform()
            viewport.camera_path = self.camera.GetPath()
            viewport.resolution = tuple(cfg['resolution'])
            self.update(None)
            self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self.update, name='DAMVI visual follow camera')

        def update(self, _event):
            try:
                if controller.body is None:
                    return
                pose = controller.body.get_transforms()[0]
                x, y, z, w = map(float, pose[3:])
                yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                angle = yaw + math.pi + math.radians(float(cfg['rear_side_angle_deg']))
                distance = float(cfg['horizontal_distance_m'])
                eye = Gf.Vec3d(float(pose[0])+distance*math.cos(angle),
                               float(pose[1])+distance*math.sin(angle),
                               float(cfg['height_above_ground_m']))
                target = Gf.Vec3d(float(pose[0]), float(pose[1]),
                                 float(pose[2])+float(cfg['look_at_height_above_chassis_m']))
                matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    self.xform.Set(matrix)
                now = time.time()
                self.latest = dict(wall_time_unix_s=now, sim_time_s=float(controller.elapsed),
                    camera_position_m=list(eye), target_position_m=list(target),
                    chassis_position_m=[float(v) for v in pose[:3]],
                    camera_distance_m=float((eye-target).GetLength()))
                self.samples.append(self.latest)
                self.samples = self.samples[-6000:]
                if now-self.last_write >= 1.:
                    self.last_write = now
                    self.output.write_text(json.dumps(dict(error=self.error, config=cfg,
                        previous_camera=self.previous_camera, camera_path=str(self.camera.GetPath()),
                        latest=self.latest, physics_changed=False, poses_written=False), indent=2)+'\n')
            except Exception as exc:
                self.error = repr(exc)
                self.subscription = None
                print('Follow camera stopped:', self.error)

        def close(self, restore=False):
            self.subscription = None
            for layer, visible in self.overlays:
                layer.visible = visible
            if restore:
                viewport.camera_path = self.previous_camera
                viewport.resolution = self.previous_resolution

    camera = FollowCamera()
    info = dict(camera_path=str(viewport.camera_path), resolution=list(viewport.resolution),
                window={k: getattr(window,k) for k in ('position_x','position_y','width','height')
                        if hasattr(window,k)})
    frame = window.frame
    info['frame'] = {k: getattr(frame,k) for k in ('screen_position_x','screen_position_y',
        'computed_width','computed_height') if hasattr(frame,k)}
    fw, fh = float(frame.computed_width), float(frame.computed_height)
    aspect = cfg['resolution'][0]/cfg['resolution'][1]
    rw, rh = min(fw, fh*aspect), min(fh, fw/aspect)
    info['x11_window_crop_xywh'] = [int(round(frame.screen_position_x+(fw-rw)/2)),
        int(round(frame.screen_position_y+(fh-rh)/2)), int(rw)//2*2, int(rh)//2*2]
    (controller.output_dir/'follow_camera_viewport.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps(info, indent=2))
    return camera


if '_damvi_follow_camera' in globals():
    _damvi_follow_camera.close()
_damvi_follow_camera = _install_damvi_recording_camera()
