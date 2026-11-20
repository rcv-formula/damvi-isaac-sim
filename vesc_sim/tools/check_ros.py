"""End-to-end synthetic command test. Run in a dedicated ROS_DOMAIN_ID.

Start this subscriber before run.sh --ros --duration 8 --realtime.
"""
import argparse
import json
from pathlib import Path
import time
import rclpy
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from vesc_msgs.msg import VescStateStamped

p=argparse.ArgumentParser();p.add_argument('-o','--output',type=Path,required=True);a=p.parse_args()
rclpy.init();node=rclpy.create_node('check_vesc_sim')
pub=node.create_publisher(Float64,'/commands/motor/speed',10)
sim_time=[None];samples=[]
node.create_subscription(Clock,'/clock',lambda m:sim_time.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
node.create_subscription(VescStateStamped,'/sensors/core',lambda m:samples.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.state.speed)),10)
start=time.monotonic();last_clock=None
while time.monotonic()-start<120:
    rclpy.spin_once(node,timeout_sec=.01)
    t=sim_time[0]
    if t is None:continue
    if t!=last_clock:
        target=2307.0 if .5<=t<2.5 else -2307.0 if 3.5<=t<5.5 else 0.
        m=Float64();m.data=target;pub.publish(m);last_clock=t
    if t>=7.98:break
checks={
    'received':len(samples)>30,
    'positive_erpm':any(v>1000 for t,v in samples if 1.5<t<2.5),
    'negative_erpm':any(v<-1000 for t,v in samples if 4.5<t<5.5),
    'sim_timestamps_increase':all(b[0]>a[0] for a,b in zip(samples,samples[1:])),
    'last_erpm_zero':bool(samples) and abs(samples[-1][1])<5,
    'clock_completed':sim_time[0] is not None and sim_time[0]>=7.98}
out={'status':'pass' if all(checks.values()) else 'fail','checks':checks,'samples':samples}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='samples'}));node.destroy_node();rclpy.shutdown()
if out['status']!='pass':raise SystemExit(1)
