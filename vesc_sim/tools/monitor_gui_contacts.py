"""Bounded contact monitor for the existing GUI, with a pause-on-failure guard.

Execute through isaac_live_client.py. This never installs an adapter, resets or
plays the timeline, sends commands, sets poses, or changes physics parameters.
Sampling follows application frames (~30 Hz), not every physics solver step.
All imports and helpers are scoped to avoid replacing another GUI script's globals.
"""


def _launch_damvi_contact_monitor():
    import asyncio
    import json
    import math
    import time
    import traceback
    import numpy as np
    import omni.kit.app
    import live_gui

    class ContactMonitor:
        DURATION_S = 30.0
        INITIAL_EPISODE_SETTLE_S = 2.0
        MAX_PENETRATION_M = 0.003
        MAX_ROLL_PITCH_RAD = math.radians(20.0)
        MAX_ABS_SUSPENSION_M = 0.003

        def __init__(self, controller):
            self.c = controller
            self.output = controller.output_dir / 'contact_motion_check.json'
            self.samples = []
            self.task = None
            self.report = {'status': 'starting', 'output': str(self.output)}
            self.paused_by_guard = False
            self.failure_reasons = []
            self.duration_s = self.DURATION_S

        @staticmethod
        def roll_pitch(q):
            x, y, z, w = q
            return (
                math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
                math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
            )

        @staticmethod
        def cylinder_bottom(pose, radius, half_width):
            # Cylinder's local axis is Y. Its vertical support is the sum of
            # the axial support and the circular cross section's support.
            x, y, z, w = pose[3:]
            axis_z = max(-1.0, min(1.0, 2 * (y*z + w*x)))
            support_z = radius * math.sqrt(max(0.0, 1-axis_z*axis_z)) + half_width * abs(axis_z)
            return float(pose[2] - support_z)

        def sample(self):
            c = self.c
            poses = np.asarray(self.bodies.get_transforms(), dtype=float).copy()
            velocities = np.asarray(self.bodies.get_velocities(), dtype=float).copy()
            positions = np.asarray(c.art.get_dof_positions()[0], dtype=float).copy()
            joint_velocities = np.asarray(c.art.get_dof_velocities()[0], dtype=float).copy()
            chassis = poses[self.chassis_index]
            roll, pitch = self.roll_pitch(chassis[3:])
            wheel_pose = poses[self.wheel_indices]
            wheel_vel = velocities[self.wheel_indices]
            bottoms = [self.cylinder_bottom(p, r, h) for p, (r, h) in zip(wheel_pose, self.wheel_geometry)]
            output = c.native.output
            row = {
                'sim_time_s': float(c.elapsed),
                'sim_elapsed_s': float(c.elapsed-self.start_sim),
                'wall_elapsed_s': float(time.monotonic()-self.start_wall),
                'guard_active': bool(c.elapsed >= self.INITIAL_EPISODE_SETTLE_S),
                'chassis_position_m': chassis[:3].tolist(),
                'chassis_orientation_xyzw': chassis[3:].tolist(),
                'chassis_velocity_world_mps_radps': velocities[self.chassis_index].tolist(),
                'roll_pitch_deg': [math.degrees(roll), math.degrees(pitch)],
                'wheel_names': list(self.wheel_names),
                'wheel_poses_xyz_xyzw': wheel_pose.tolist(),
                'wheel_velocities_world_mps_radps': wheel_vel.tolist(),
                'wheel_bottom_z_m': bottoms,
                'min_wheel_bottom_z_m': min(bottoms),
                'suspension_positions_m': positions[self.suspension_indices].tolist(),
                'suspension_velocities_mps': joint_velocities[self.suspension_indices].tolist(),
                'suspension_limit_overrun_m': np.maximum(0., np.maximum(
                    self.suspension_limits[:, 0]-positions[self.suspension_indices],
                    positions[self.suspension_indices]-self.suspension_limits[:, 1])).tolist(),
                'steering_actual_left_right_rad': positions[c.steer_indices].tolist(),
                'steering_target_left_right_rad': np.asarray(c.position_targets[0, c.steer_indices], dtype=float).tolist(),
                'steering_input_rad': float(c.steering_input),
                'target_erpm': float(c.target),
                'target_speed_mps': float(c.target/c.config['speed_to_erpm_gain']),
                'wheel_omega_rad_s': joint_velocities[c.indices].tolist(),
                'erpm': float(c.native.erpm(joint_velocities[c.indices])),
                'applied_wheel_efforts_nm': np.asarray(c.forces[0, c.indices], dtype=float).tolist(),
                'native_motor_current_a': float(output.current),
                'native_center_transfer_torque_nm': float(output.center_torque),
                'native_center_speed_error_rad_s': float(output.center_speed_error),
                'native_center_twist_rad': float(c.native.drive.center_twist),
                'native_battery_current_a': float(output.battery_current),
                'native_electrical_brake': bool(output.electrical_brake),
                'controller_mode': str(c.mode),
                'controller_error': c.error,
            }
            finite = all(np.isfinite(x).all() for x in (poses, velocities, positions, joint_velocities,
                np.asarray(bottoms), np.asarray(c.forces)))
            if not finite:
                self.failure_reasons.append('Non-finite physical state or applied effort')
            if c.error:
                self.failure_reasons.append('Controller reported an error')
            if row['guard_active']:
                if min(bottoms) < -self.MAX_PENETRATION_M:
                    self.failure_reasons.append(f'Wheel bottom {min(bottoms):.6f} m below ground')
                if max(abs(roll), abs(pitch)) > self.MAX_ROLL_PITCH_RAD:
                    self.failure_reasons.append(f'Chassis tilt exceeds 20 degrees: {row["roll_pitch_deg"]}')
                if np.max(np.abs(positions[self.suspension_indices])) > self.MAX_ABS_SUSPENSION_M:
                    self.failure_reasons.append('Suspension magnitude exceeds 3 mm')
            return row

        def summarize(self, status):
            rows = self.samples
            guarded = [r for r in rows if r['guard_active']]
            span = rows[-1]['sim_elapsed_s'] if rows else 0.
            wall = time.monotonic()-self.start_wall
            xyz = np.asarray([r['chassis_position_m'] for r in rows], dtype=float)
            self.report = {
                'status': status,
                'failure_reasons': self.failure_reasons,
                'paused_by_guard': self.paused_by_guard,
                'read_only_except_failure_pause': True,
                'commands_published': False,
                'poses_written': False,
                'physics_settings_changed': False,
                'sampling': 'one snapshot per distinct simulation time at application update; not every physics step',
                'target_duration_sim_s': self.duration_s,
                'measured_duration_sim_s': span,
                'wall_duration_s': wall,
                'average_realtime_factor': span/wall if wall else None,
                'sample_count': len(rows),
                'guarded_sample_count': len(guarded),
                'average_sample_rate_hz': (len(rows)-1)/span if span and rows else None,
                'physics_hz': self.c.config['physics_hz'],
                'thresholds': {
                    'initial_episode_settle_s': self.INITIAL_EPISODE_SETTLE_S,
                    'wheel_bottom_z_min_m': -self.MAX_PENETRATION_M,
                    'absolute_roll_pitch_max_deg': math.degrees(self.MAX_ROLL_PITCH_RAD),
                    'absolute_suspension_max_m': self.MAX_ABS_SUSPENSION_M,
                },
                'wheel_geometry_radius_halfwidth_m': [list(v) for v in self.wheel_geometry],
                'min_wheel_bottom_z_m': min((r['min_wheel_bottom_z_m'] for r in guarded), default=None),
                'max_wheel_penetration_m': max((max(0., -r['min_wheel_bottom_z_m']) for r in guarded), default=None),
                'max_absolute_roll_pitch_deg': max((max(abs(v) for v in r['roll_pitch_deg']) for r in guarded), default=None),
                'max_absolute_suspension_m': max((max(abs(v) for v in r['suspension_positions_m']) for r in guarded), default=None),
                'max_suspension_limit_overrun_m': max((max(r['suspension_limit_overrun_m']) for r in guarded), default=None),
                'max_chassis_speed_mps': max((float(np.linalg.norm(r['chassis_velocity_world_mps_radps'][:3])) for r in rows), default=None),
                'chassis_xy_distance_m': float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum()) if len(xyz)>1 else 0.,
                'max_abs_wheel_omega_rad_s': max((max(abs(v) for v in r['wheel_omega_rad_s']) for r in rows), default=None),
                'max_abs_applied_wheel_effort_nm': max((max(abs(v) for v in r['applied_wheel_efforts_nm']) for r in rows), default=None),
                'max_abs_target_speed_mps': max((abs(r['target_speed_mps']) for r in rows), default=None),
                'max_abs_steering_input_rad': max((abs(r['steering_input_rad']) for r in rows), default=None),
                'output': str(self.output),
                'samples': rows,
            }
            # allow_nan=False prevents a malformed diagnostic result on a fault.
            # Finite-state failures are represented with null values in the raw data.
            def safe(value):
                if isinstance(value, float) and not math.isfinite(value): return None
                if isinstance(value, list): return [safe(x) for x in value]
                if isinstance(value, dict): return {k: safe(v) for k, v in value.items()}
                return value
            self.report = safe(self.report)
            self.output.write_text(json.dumps(self.report, indent=2, allow_nan=False)+'\n')

        async def run(self):
            self.start_wall = time.monotonic()
            self.start_sim = float(self.c.elapsed)
            self.wheel_geometry = []
            try:
                if not self.c.timeline.is_playing() or self.c.art is None:
                    raise RuntimeError('An already-playing initialized controller is required')
                self.wheel_names = ('FL', 'FR', 'RL', 'RR')
                root = self.c.config['scene']['vehicle_root']
                self.bodies = self.c.view.create_rigid_body_view(root+'/*')
                paths = list(self.bodies.prim_paths)
                self.chassis_index = paths.index(self.c.config['scene']['chassis'])
                self.wheel_indices = [paths.index(root+'/Wheel_'+n) for n in self.wheel_names]
                for name in self.wheel_names:
                    collider = self.c.stage.GetPrimAtPath(root+'/Wheel_'+name+'/TireCollision')
                    if collider.GetTypeName() != 'Cylinder' or collider.GetAttribute('axis').Get() != 'Y':
                        raise RuntimeError('Analytical bounds require existing Y-axis Cylinder tire colliders')
                    radius = float(collider.GetAttribute('radius').Get())
                    halfwidth = float(collider.GetAttribute('height').Get())*.5
                    self.wheel_geometry.append((radius, halfwidth))
                names = list(self.c.art.shared_metatype.dof_names)
                self.suspension_indices = [names.index('Suspension_'+n) for n in self.wheel_names]
                self.suspension_limits = np.asarray(self.c.art.get_dof_limits()[0, self.suspension_indices], dtype=float)
                self.summarize('running')
                app = omni.kit.app.get_app()
                last_sim = None
                while self.c.elapsed-self.start_sim < self.duration_s:
                    if time.monotonic()-self.start_wall > max(30., self.duration_s*4+10):
                        raise RuntimeError('Wall timeout before the requested simulation interval completed')
                    if self.c.elapsed < self.start_sim:
                        raise RuntimeError('Simulation time reset during monitoring')
                    if not self.c.timeline.is_playing():
                        raise RuntimeError('Timeline stopped or paused during monitoring')
                    if self.c.elapsed != last_sim:
                        self.samples.append(self.sample())
                        last_sim = self.c.elapsed
                    if self.failure_reasons:
                        self.c.timeline.pause()
                        self.paused_by_guard = True
                        self.summarize('fail')
                        break
                    await app.next_update_async()
                else:
                    self.samples.append(self.sample())
                    if self.failure_reasons:
                        self.c.timeline.pause()
                        self.paused_by_guard = True
                    self.summarize('fail' if self.failure_reasons else 'pass')
            except Exception:
                self.failure_reasons.append(traceback.format_exc())
                # An inspection error also pauses a playing simulation so the
                # operator cannot mistake a failed monitor for a healthy guard.
                if self.c.timeline.is_playing():
                    self.c.timeline.pause()
                    self.paused_by_guard = True
                self.summarize('fail')
            print('[CONTACT_MONITOR] '+json.dumps({k: v for k, v in self.report.items() if k!='samples'}), flush=True)

    prior = globals().get('_damvi_contact_monitor')
    if prior is not None and prior.task is not None and not prior.task.done():
        raise RuntimeError('A contact monitor is already running')
    monitor = ContactMonitor(live_gui.controller)
    monitor.task = asyncio.ensure_future(monitor.run())
    return monitor


_damvi_contact_monitor = _launch_damvi_contact_monitor()
print('[CONTACT_MONITOR] Armed for 30 simulated seconds; no commands sent.')
