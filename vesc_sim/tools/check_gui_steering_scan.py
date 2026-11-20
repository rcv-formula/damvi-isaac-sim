"""Exercise physical front-wheel DOFs and scan metadata in the current GUI.

Commands have zero linear speed, so this sweep does not drive around the map.
"""
import argparse,json,math,sys,time
from pathlib import Path
from gui_trace import flush_trace
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from steering import SteeringController
p=argparse.ArgumentParser();p.add_argument('-o','--output',type=Path,required=True);a=p.parse_args()
cfg=yaml.safe_load((ROOT/'config/steering.yaml').read_text())
reference_cfg=dict(cfg);reference_cfg['actuator_time_constant_s']=0.;reference_cfg['actuator_rate_limit_rad_s']=1e6
reference=SteeringController(reference_cfg)
clock=[None];origin=[None];last=None;scans=[]
rclpy.init();node=rclpy.create_node('check_steering_scan_current_gui')
node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
def receive(m):
    finite=[float(v) for v in m.ranges if math.isfinite(v)]
    scans.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,
        'count':len(m.ranges),'angle_min':m.angle_min,'angle_max':m.angle_max,'increment':m.angle_increment,
        'scan_time':m.scan_time,'time_increment':m.time_increment,'finite_returns':len(finite),
        'minimum':min(finite) if finite else None,'maximum':max(finite) if finite else None,
        'has_nan':any(math.isnan(v) for v in m.ranges)})
node.create_subscription(LaserScan,'/scan',receive,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT))
pub=node.create_publisher(Twist,cfg['input_topic'],10)
commands=[0.,.3,.6,.85,.7,0.,-.3,-.6,-.85,-.7,0.];period=.8
start=time.monotonic()
while time.monotonic()-start<120:
    rclpy.spin_once(node,timeout_sec=.002)
    if clock[0] is None:continue
    if origin[0] is None:origin[0]=clock[0]
    elapsed=clock[0]-origin[0]
    if clock[0]!=last:
        index=min(int(elapsed/period),len(commands)-1)
        msg=Twist();msg.angular.z=commands[index];pub.publish(msg);last=clock[0]
    if elapsed>len(commands)*period+.5:break
flush_trace(ROOT)
trace=json.loads((ROOT/'outputs/live_gui/trace.json').read_text())
observations=[]
for i,command in enumerate(commands[:-1]):
    rows=[r for r in trace if origin[0]+i*period+.5<r['time']<origin[0]+(i+1)*period-.05]
    expected=reference.step(command,1.)
    if rows:
        actual=np.median([r['steering_actual_left_right_rad'] for r in rows],axis=0)
        error=float(np.max(np.abs(actual-[expected.left,expected.right])))
        observations.append({'input_rad':command,'expected_left_right_rad':[expected.left,expected.right],
            'actual_left_right_rad':actual.tolist(),'max_error_deg':float(np.rad2deg(error)),
            'max_chassis_speed_mps':max(float(np.linalg.norm(r['velocity'][:2])) for r in rows)})
checks={'clock_completed':clock[0] is not None and clock[0]-origin[0]>=len(commands)*period,
        'all_sweep_phases_measured':len(observations)==len(commands)-1,
        'physical_steering_tracks_map':len(observations)==len(commands)-1 and all(o['max_error_deg']<1.5 for o in observations),
        'stationary_sweep':bool(observations) and max(o['max_chassis_speed_mps'] for o in observations)<.1,
        'scan_received':len(scans)>50,
        'scan_beams':bool(scans) and all(s['count']==1081 for s in scans),
        'scan_frame':bool(scans) and all(s['frame']=='laser' for s in scans),
        'scan_valid_ranges':bool(scans) and all(not s['has_nan'] and s['finite_returns']>10 for s in scans),
        'scan_period_40hz':len(scans)>2 and abs(float(np.median(np.diff([s['stamp'] for s in scans])))-.025)<1e-6,
        'scan_geometry':bool(scans) and abs(scans[-1]['angle_min']+3*math.pi/4)<1e-6 and abs(scans[-1]['increment']-math.pi/720)<1e-8}
by_command={o['input_rad']:o['actual_left_right_rad'] for o in observations}
checks['physical_servo_saturation']=all(a in by_command and b in by_command and
    float(np.max(np.abs(np.asarray(by_command[a])-by_command[b])))<math.radians(.3)
    for a,b in ((.85,.7),(-.85,-.7)))
out={'status':'pass' if all(checks.values()) else 'fail','checks':checks,'observations':observations,
     'scans_received':len(scans),'first_scan':scans[0] if scans else None,'last_scan':scans[-1] if scans else None}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2));node.destroy_node();rclpy.shutdown()
if out['status']!='pass':raise SystemExit(1)
