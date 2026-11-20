"""Torque controller for the already-open GUI; never opens/saves a USD.

Load with tools/install_gui.py through the GUI's localhost Python Server.
"""
import asyncio
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
import numpy as np
import yaml
import omni.kit.app
import omni.usd
import omni.timeline
import omni.physx
import omni.physics.tensors
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from native import NativeController
from steering import load_configuration,SteeringController
from steering_scene import prepare_front_steering
from wheel_collision import prepare_wheel_collisions,apply_wheel_angular_limits
from tire_physics import prepare_tire_physics,TirePhysics
from lidar import LidarSensor
from imu import ImuSensor
from tf_config import FrameTree,StaticFrames

HERE=Path(__file__).resolve().parent


class LiveController:
    def __init__(self):
        import carb.settings
        self.headless=bool(carb.settings.get_settings().get('/damvi/headless'))
        self.stage=omni.usd.get_context().get_stage()
        self.config=self.load_stage_configuration(HERE/'config/vesc_sim.yaml')
        self.native=NativeController(self.config)
        self.steering=SteeringController(self.config['_steering'])
        self.steering_input=0.
        self.timeline=omni.timeline.get_timeline_interface()
        self.elapsed=0.;self.target=0.;self.brake=None;self.demo_start=None
        self.mode='stop';self.error=None;self.art=None;self.view=None;self.body=None
        self.pre=None;self.post=None;self.frames=None;self.bridge=None
        self.lidar=None
        self.tires=None
        self.tire_geometry={'enabled':False,'mode':'not_initialized'}
        self.imu=None;self.static_frames=None;self.frame_tree=None
        self.timeline_events=None
        self.publish_at=0.;self.last_write=0.;self.rows=[];self.busy=False
        self.last_trace_write=0.;self.realtime_factor=None
        self.rtf_wall=time.monotonic();self.rtf_sim=0.
        self.output_dir=HERE/'outputs/live_gui';self.output_dir.mkdir(parents=True,exist_ok=True)
        self.config_path=HERE/'config/vesc_sim.yaml'
        self.config_sha256=hashlib.sha256(self.config_path.read_bytes()).hexdigest()
        self.config_load_count=1

    def load_stage_configuration(self, path):
        track=self.stage.GetPrimAtPath('/World/TrackEnvironment')
        dataset=track.GetCustomDataByKey('damvi:dataset') if track else None
        return load_configuration(path,surface_dataset=dataset)

    def stop(self):
        self.mode='stop';self.demo_start=None;self.target=0.;self.brake=None

    def demo(self):
        self.mode='demo';self.demo_start=self.elapsed;self.target=0.;self.brake=None
        self.timeline.play()

    def ros_mode(self):
        if self.bridge is not None:self.mode='ros';self.demo_start=None;self.timeline.play()

    async def start(self):
        self.timeline.stop()
        app=omni.kit.app.get_app()
        for _ in range(3):await app.next_update_async()
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        self.apply_runtime(self.config)
        chassis=self.config['scene']['chassis'];root=self.config['scene']['vehicle_root']
        if not self.stage.GetPrimAtPath(chassis):raise RuntimeError('Expected RC car is absent from open GUI')
        self.steering_geometry=prepare_front_steering(self.stage,root,self.config['_steering'])
        self.wheel_collision_geometry=prepare_wheel_collisions(self.stage,root,self.config['_runtime'])
        self.tire_geometry=prepare_tire_physics(self.stage,root,self.config['_tires'])
        self.tire_enabled_configuration=bool(self.config['_tires'].get('enabled',False))
        self.wheel_collision_configuration=(self.config['_runtime'].get('wheel_collision_approximation','analytic'),
            self.config['_runtime'].get('wheel_collision_segments',64))
        self.original_friction={}
        for n in ('FL','FR','RL','RR'):
            prim=self.stage.GetPrimAtPath(root+'/Joints/Axle_'+n)
            joint=PhysxSchema.PhysxJointAPI.Apply(prim)
            self.original_friction[n]=joint.GetJointFrictionAttr().Get()
            joint.CreateJointFrictionAttr(0.)
            if prim.HasAPI(UsdPhysics.DriveAPI,'angular'):
                drive=UsdPhysics.DriveAPI(prim,'angular')
                drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(0.);drive.CreateMaxForceAttr(0.)
        scenes=[p for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1:raise RuntimeError('Expected one PhysicsScene')
        PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateTimeStepsPerSecondAttr(self.config['physics_hz'])
        frame=UsdGeom.Xform.Define(self.stage,chassis+'/base_link')
        transform=UsdGeom.XformCommonAPI(frame)
        transform.SetTranslate(Gf.Vec3d(*self.config['scene']['rear_axle_offset_m']))
        # This GUI was looping a single animation frame; let physics accumulate time.
        self.stage.SetStartTimeCode(0);self.stage.SetEndTimeCode(3600*self.stage.GetTimeCodesPerSecond())
        self.timeline.set_start_time(0);self.timeline.set_end_time(3600);self.timeline.set_looping(False)
        self.timeline.set_current_time(0)
        try:
            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.ros2.bridge',True)
            await app.next_update_async()
            prefix=Path('/home/shin/damvi_vesc_ros_ws/install/vesc_msgs')
            sys.path.insert(0,str(prefix/'lib/python3.12/site-packages'))
            # The current GUI predates this message build, so preload custom types.
            for name in ('rosidl_generator_c','rosidl_typesupport_introspection_c','rosidl_typesupport_fastrtps_c','rosidl_typesupport_c','rosidl_generator_py'):
                ctypes.CDLL(str(prefix/f'lib/libvesc_msgs__{name}.so'),mode=ctypes.RTLD_GLOBAL)
            from ros_bridge import RosBridge
            self.bridge=RosBridge(self.config)
        except Exception:
            self.ros_error=traceback.format_exc()
            (self.output_dir/'ros_error.txt').write_text(self.ros_error)
        self.setup_sensors()
        physx=omni.physx.get_physx_interface()
        self.pre=physx.subscribe_physics_on_step_events(self.before_step,True,0)
        self.post=physx.subscribe_physics_on_step_events(self.after_step,False,0)
        self.frames=app.get_update_event_stream().create_subscription_to_pop(self.on_frame,name='damvi_vesc_live_gui')
        self.timeline_events=self.timeline.get_timeline_event_stream().create_subscription_to_pop(self.on_timeline,name='damvi_vesc_config_reload')
        self.timeline.play()
        # Wait for PhysX to insert the articulation, without blocking GUI frames.
        for _ in range(120):
            await app.next_update_async()
            if self.art is not None:break
        if self.art is None:raise RuntimeError('GUI articulation did not initialize')
        self.mode='ros' if self.bridge else 'stop'
        self.write_status()
        print('[VESC_GUI] Attached to existing GUI; ROS input, no custom UI',flush=True)

    def on_timeline(self,event):
        if int(event.type)!=int(omni.timeline.TimelineEventType.PLAY):return
        try:
            text=self.config_path.read_bytes()
            config=self.load_stage_configuration(self.config_path)
            hz=config['physics_hz']
            if hz<=0 or config['controller_hz']%hz or config['erpm_sign'] not in (-1,1) or config['speed_to_erpm_gain']<=0 or config['speed_to_erpm_offset']!=0:
                raise ValueError('Invalid simulation rate, ERPM gain/sign/offset')
            native=NativeController(config)
            steering=SteeringController(config['_steering'])
            frame_tree=FrameTree(config['_tf_path'])
            self.apply_runtime(config)
            if (config['_runtime'].get('wheel_collision_approximation','analytic'),
                    config['_runtime'].get('wheel_collision_segments',64))!=self.wheel_collision_configuration:
                raise ValueError('Wheel collision geometry changes require reinstalling the adapter while stopped')
            if bool(config['_tires'].get('enabled',False))!=self.tire_enabled_configuration:
                raise ValueError('Changing tire force mode requires reinstalling the adapter while stopped')
            if any(abs(config['_steering'][key]-self.steering_geometry[name])>.001 for key,name in (
                    ('simulation_wheelbase_m','wheelbase_m'),('simulation_front_track_m','front_track_m'))):
                raise ValueError('Steering geometry must match the existing USD; changing geometry requires scene preparation')
            scenes=[p for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            PhysxSchema.PhysxSceneAPI.Apply(scenes[0]).CreateTimeStepsPerSecondAttr(hz)
            if config['scene']!=self.config['scene']:
                raise ValueError('Scene paths/geometry configuration changes require reinstalling the adapter')
            if self.bridge:
                if any(config['ros'][n]!=self.config['ros'][n] for n in ('command_topic','brake_topic','output_topic')) or config['_steering']['input_topic']!=self.config['_steering']['input_topic']:
                    from ros_bridge import RosBridge
                    self.close_sensors()
                    self.bridge.close();self.bridge=RosBridge(config)
                else:
                    self.bridge.config=config;self.bridge.received=None;self.bridge.steering_received=None;self.bridge.steering_input=0.
            self.config=config;self.native=native;self.steering=steering;self.steering_input=0.
            if self.art is not None and self.tire_enabled_configuration:
                self.tires=TirePhysics(self.stage,self.view,config,config['_tires'])
            self.setup_sensors(frame_tree)
            for name in ('FL','FR'):
                drive=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(config['scene']['vehicle_root']+'/Joints/Steer_'+name),'angular')
                drive.CreateStiffnessAttr(math.radians(config['_steering']['joint_stiffness']));drive.CreateDampingAttr(math.radians(config['_steering']['joint_damping']))
                drive.CreateMaxForceAttr(config['_steering']['joint_max_torque_nm'])
            self.config_sha256=hashlib.sha256(text).hexdigest();self.config_load_count+=1
            self.stop();self.mode='ros' if self.bridge else 'stop';self.error=None
            self.write_status()
            print(f'[VESC_GUI] Read {self.config_path} at Play; gain={config["speed_to_erpm_gain"]}',flush=True)
        except Exception:
            self.error=traceback.format_exc()
            if self.art:
                self.forces.fill(0);self.art.set_dof_actuation_forces(self.forces,self.art_indices)
            self.timeline.pause();self.write_status()

    def apply_runtime(self,config):
        import carb.settings
        runtime=config['_runtime']
        apply_wheel_angular_limits(self.stage,config['scene']['vehicle_root'],runtime)
        threads=runtime['physics_worker_threads']
        if not isinstance(threads,int) or not 0<=threads<=256:
            raise ValueError('physics_worker_threads must be an integer in [0, 256]')
        if runtime['broadphase'] not in ('MBP','SAP','GPU'):
            raise ValueError('Unknown broadphase')
        for name in ('status_interval_s','trace_snapshot_interval_s'):
            if not math.isfinite(runtime[name]) or runtime[name]<0 or (name=='status_interval_s' and runtime[name]==0):
                raise ValueError('Invalid diagnostics interval')
        render_hz=float(runtime['render_hz'])
        if not math.isfinite(render_hz) or not 0<render_hz<=240:
            raise ValueError('render_hz must be in (0, 240]')
        minimum=float(runtime['min_frame_rate'])
        if not math.isfinite(minimum) or not 0<minimum<render_hz:
            raise ValueError('min_frame_rate must be positive and below render_hz')
        settings=carb.settings.get_settings()
        if runtime.get('wheel_collision_approximation')=='analytic':
            settings.set('/physics/collisionApproximateCylinders',False)
        settings.set('/persistent/simulation/minFrameRate',minimum)
        # Python physics callbacks use the Carbonite dispatcher. Switching an
        # active GUI to the internal worker dispatcher can deadlock callbacks.
        initial=(threads,runtime['broadphase'])
        if self.timeline.is_stopped():
            settings.set('/physics/physxDispatcher',False)
            settings.set('/persistent/physics/numThreads',threads)
            omni.physx.get_physx_interface().set_thread_count(threads)
            for prim in self.stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    PhysxSchema.PhysxSceneAPI.Apply(prim).CreateBroadphaseTypeAttr(runtime['broadphase'])
            self.applied_physics_runtime=initial
        elif initial!=getattr(self,'applied_physics_runtime',None):
            raise ValueError('Worker count/broadphase changes require reinstalling the adapter while stopped')
        if render_hz!=getattr(self,'applied_render_hz',None):
            from isaacsim.core.rendering_manager import RenderingManager
            settings.set('/app/runLoops/main/rateLimitEnabled',True)
            settings.set('/app/runLoops/main/rateLimitFrequency',render_hz)
            # Public API configures the application/timeline dt coherently;
            # the physics scene's independent 200 Hz is set separately below.
            RenderingManager.set_dt(1./render_hz)
            self.applied_render_hz=render_hz
        # Isaac's Play throttling otherwise forces a fixed amount of simulated
        # time per rendered frame, making a missed GUI frame slow the vehicle.
        realtime=runtime.get('realtime_wall_clock',True)
        settings.set('/exts/isaacsim.core.throttling/enable_manualmode',not realtime)
        settings.set('/app/player/useFixedTimeStepping',not realtime)
        from omni.kit.loop import _loop
        _loop.acquire_loop_interface().set_manual_mode(not realtime)

    def setup_sensors(self,frame_tree=None):
        if self.bridge is None:return
        self.frame_tree=frame_tree if frame_tree is not None else FrameTree(self.config['_tf_path'])
        if self.static_frames:self.static_frames.reload(self.frame_tree)
        else:self.static_frames=StaticFrames(self.bridge.node,self.frame_tree)
        if self.lidar:
            self.lidar.path=Path(self.config['_lidar_path']);self.lidar.reload(frame_tree=self.frame_tree)
        else:
            self.lidar=LidarSensor(self.stage,self.bridge.node,self.config['_lidar_path'],
                chassis_prim=self.config['scene']['chassis'],vehicle_root=self.config['scene']['vehicle_root'],
                base_link_offset_m=self.config['scene']['rear_axle_offset_m'],frame_tree=self.frame_tree)
        if self.imu:
            self.imu.path=Path(self.config['_imu_path']);self.imu.reload(frame_tree=self.frame_tree,physics_hz=self.config['physics_hz'])
        else:
            self.imu=ImuSensor(self.stage,self.bridge.node,self.config['_imu_path'],
                base_link_offset_m=self.config['scene']['rear_axle_offset_m'],frame_tree=self.frame_tree,
                physics_hz=self.config['physics_hz'])
        if self.body is not None:
            self.imu.set_center_of_mass_offset_m(self.body.get_coms()[0,:3]*UsdGeom.GetStageMetersPerUnit(self.stage))

    def close_sensors(self):
        if self.lidar:self.lidar.close();self.lidar=None
        if self.imu:self.imu.close();self.imu=None
        if self.static_frames:self.static_frames.close();self.static_frames=None
        self.frame_tree=None

    def initialize_views(self):
        self.view=omni.physics.tensors.create_simulation_view('numpy',stage_id=omni.usd.get_context().get_stage_id())
        self.view.set_subspace_roots('/')
        self.art=self.view.create_articulation_view(self.config['scene']['chassis'])
        if self.art.count!=1:raise RuntimeError('Expected one live articulation')
        names=list(self.art.shared_metatype.dof_names)
        self.indices=[names.index('Axle_'+n) for n in ('FL','FR','RL','RR')]
        self.steer_indices=[names.index('Steer_'+n) for n in ('FL','FR')]
        self.forces=np.zeros_like(self.art.get_dof_velocities(),dtype=np.float32)
        self.position_targets=np.asarray(self.art.get_dof_position_targets(),dtype=np.float32).copy()
        self.art_indices=np.asarray([0],dtype=np.uint32)
        self.body=self.view.create_rigid_body_view(self.config['scene']['chassis'])
        self.tires=TirePhysics(self.stage,self.view,self.config,self.config['_tires']) if self.tire_enabled_configuration else None
        if self.imu:self.imu.set_center_of_mass_offset_m(self.body.get_coms()[0,:3]*UsdGeom.GetStageMetersPerUnit(self.stage))

    def before_step(self,dt):
        if self.art is None or self.error:return
        try:
            if self.mode=='demo':
                age=self.elapsed-self.demo_start
                if age>=12:self.stop()
                else:
                    cycle=age%6
                    speed=1.2 if 1<=cycle<2 else -1.2 if 3<=cycle<4 else 0.
                    self.target=speed*self.config['speed_to_erpm_gain']
            omega=np.asarray(self.art.get_dof_velocities()[0,self.indices],dtype=float)
            out=self.native.step(self.target,omega,float(dt),self.brake)
            steering=self.steering.step(self.steering_input,float(dt))
            self.position_targets[0,self.steer_indices]=[steering.left,steering.right]
            self.art.set_dof_position_targets(self.position_targets,self.art_indices)
            self.forces.fill(0);self.forces[0,self.indices]=np.asarray(out.torque,dtype=np.float32)
            self.art.set_dof_actuation_forces(self.forces,self.art_indices)
            if self.tires:self.tires.step(float(dt))
        except Exception:
            self.error=traceback.format_exc();self.timeline.pause();self.write_status()

    def after_step(self,dt):
        self.elapsed+=float(dt)
        if self.art is None or self.error:return
        try:
            omega=np.asarray(self.art.get_dof_velocities()[0,self.indices],dtype=float)
            erpm=self.native.erpm(omega)
            pose=self.body.get_transforms()[0];velocity=self.body.get_velocities()[0]
            if self.lidar:self.lidar.update(self.elapsed,pose)
            if self.imu:self.imu.update(self.elapsed,pose,velocity)
            if self.elapsed+1e-9>=self.publish_at:
                if self.bridge:self.bridge.publish(self.elapsed,erpm)
                pos=pose[:3];vel=velocity[:3]
                self.rows.append({'time':self.elapsed,'erpm':float(erpm),'target_erpm':self.target,
                    'current_a':self.native.output.current,'position':pos.tolist(),'velocity':vel.tolist(),
                    'orientation_xyzw':pose[3:].tolist(),'angular_velocity_rad_s':velocity[3:].tolist(),
                    'wheel_omega':omega.tolist(),'mode':self.mode,'steering_input_rad':self.steering_input,
                    'steering_center_target_rad':self.steering.output.center,
                    'steering_target_left_right_rad':[self.steering.output.left,self.steering.output.right],
                    'steering_actual_left_right_rad':self.art.get_dof_positions()[0,self.steer_indices].tolist()})
                self.rows=self.rows[-2000:]
                self.publish_at=self.elapsed+1/self.config['ros']['publish_rate_hz']
        except Exception:
            self.error=traceback.format_exc();self.timeline.pause();self.write_status()

    def on_frame(self,event):
        if self.timeline.is_stopped() and self.art is not None:
            self.tires=None
            self.stop();self.art=None;self.body=None;self.view=None
            self.native.reset();self.elapsed=0.;self.publish_at=0.;self.rows=[]
            self.steering=SteeringController(self.config['_steering']);self.steering_input=0.
            if self.lidar:self.lidar.reset()
            if self.imu:self.imu.reset()
        if self.timeline.is_playing() and self.art is None and not self.error:
            try:self.initialize_views()
            except Exception:self.art=None
        if self.bridge:
            command,brake=self.bridge.spin(self.elapsed)
            if self.mode=='ros':self.target=command;self.brake=brake
            self.steering_input=self.bridge.steering_input
        now=time.monotonic()
        if self.elapsed<self.rtf_sim:
            self.rtf_wall=now;self.rtf_sim=self.elapsed;self.realtime_factor=None
        if now-self.rtf_wall>=2.:
            self.realtime_factor=(self.elapsed-self.rtf_sim)/(now-self.rtf_wall)
            self.rtf_wall=now;self.rtf_sim=self.elapsed
        if now-self.last_write>self.config['_runtime']['status_interval_s']:self.write_status()

    def write_status(self,force_trace=False):
        self.last_write=time.monotonic()
        report={'status':'error' if self.error else 'running','stage':self.stage.GetRootLayer().identifier,
            'gui_session':not self.headless,'headless_session':self.headless,
            'mode':self.mode,'elapsed':self.elapsed,'ros_connected':self.bridge is not None,
            'error':self.error,'target_erpm':self.target,'original_joint_friction':getattr(self,'original_friction',{}),
            'latest':self.rows[-1] if self.rows else None}
        report.update(config_path=str(self.config_path),config_sha256=self.config_sha256,
            config_load_count=self.config_load_count,speed_to_erpm_gain=self.config['speed_to_erpm_gain'],custom_ui=False)
        report.update(steering_config_path=self.config['_steering_path'],
            steering_config_sha256=hashlib.sha256(Path(self.config['_steering_path']).read_bytes()).hexdigest(),
            steering_geometry=getattr(self,'steering_geometry',None))
        report['lidar']=self.lidar.status() if self.lidar else None
        report['imu']=self.imu.status() if self.imu else None
        report['tf']=self.frame_tree.status() if self.frame_tree else None
        report['physics_hz']=self.config['physics_hz']
        report['realtime_factor']=self.realtime_factor
        report['runtime']=self.config['_runtime']
        report['surface_friction']=self.config.get('_surface')
        report['tires']=self.tires.status() if self.tires else self.tire_geometry
        report['wheel_collision']=getattr(self,'wheel_collision_geometry',None)
        (self.output_dir/'status.json').write_text(json.dumps(report,indent=2))
        trace_interval=self.config['_runtime']['trace_snapshot_interval_s']
        if force_trace or (trace_interval>0 and self.last_write-self.last_trace_write>=trace_interval):
            (self.output_dir/'trace.json').write_text(json.dumps(self.rows))
            self.last_trace_write=self.last_write

    def close(self):
        self.stop()
        if self.art:
            self.forces.fill(0)
            try:self.art.set_dof_actuation_forces(self.forces,self.art_indices)
            except Exception:pass
        self.pre=None;self.post=None;self.frames=None;self.timeline_events=None
        self.close_sensors()
        if self.bridge:self.bridge.close();self.bridge=None
        self.art=None;self.body=None;self.view=None
        self.tires=None


async def install():
    global controller
    controller=LiveController()
    try:await controller.start()
    except Exception:
        controller.error=traceback.format_exc();controller.write_status()
        print(controller.error)
