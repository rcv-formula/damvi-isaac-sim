"""Verify the current GUI's ROS command/telemetry path using relative /clock."""
import argparse,json,time
from pathlib import Path
import rclpy
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from vesc_msgs.msg import VescStateStamped
p=argparse.ArgumentParser();p.add_argument('-o','--output',type=Path,required=True);a=p.parse_args()
rclpy.init();node=rclpy.create_node('check_live_vesc_sim')
clock=[None];origin=[None];rows=[];last=None
node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
def receive(m):
    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    rows.append((stamp,m.state.speed))
node.create_subscription(VescStateStamped,'/sensors/core',receive,10)
pub=node.create_publisher(Float64,'/commands/motor/speed',10)
wall=time.monotonic()
while time.monotonic()-wall<90:
    rclpy.spin_once(node,timeout_sec=.005)
    if clock[0] is None:continue
    if origin[0] is None:origin[0]=clock[0]
    t=clock[0]-origin[0]
    if clock[0]!=last:
        m=Float64();m.data=1845.6 if .5<=t<1.5 else -1845.6 if 2.5<=t<3.5 else 0.
        pub.publish(m);last=clock[0]
    if t>=5.5:break
relative=[(t-origin[0],v) for t,v in rows] if origin[0] is not None else []
checks={'clock_completed':clock[0] is not None and clock[0]-origin[0]>=5.5,
    'positive_erpm':any(v>1000 for t,v in relative if 1<t<1.5),
    'negative_erpm':any(v<-1000 for t,v in relative if 3<t<3.5),
    'stopped':bool(relative) and abs(relative[-1][1])<5,
    'timestamps_increase':all(b[0]>a[0] for a,b in zip(relative,relative[1:]))}
out={'status':'pass' if all(checks.values()) else 'fail','checks':checks,'relative_samples':relative}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='relative_samples'}))
node.destroy_node();rclpy.shutdown()
if out['status']!='pass':raise SystemExit(1)
