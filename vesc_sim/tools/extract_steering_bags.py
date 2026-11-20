"""Extract calibration signals from MCAP without publishing ROS messages."""
import argparse,json
from pathlib import Path
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np

TOPICS={'/imu/data':'imu','/imu/data_raw':'imu_raw','/drive':'drive',
        '/commands/servo/position':'servo','/sensors/core':'core',
        '/odom':'odom','/odom_wheel':'wheel','/localization_pf/pose':'pf'}

def quaternion(q):return [q.x,q.y,q.z,q.w]
def stamp(msg,base):return msg.header.stamp.sec-base+msg.header.stamp.nanosec*1e-9

def extract(path,out):
    with path.open('rb') as f:
        reader=make_reader(f,decoder_factories=[DecoderFactory()])
        summary=reader.get_summary();base=summary.statistics.message_start_time//1_000_000_000
        values={name:[] for name in TOPICS.values()};frames={}
        for schema,ch,msg,d in reader.iter_decoded_messages(topics=list(TOPICS)):
            name=TOPICS[ch.topic];t=msg.log_time*1e-9-base
            if name.startswith('imu'):
                row=[t,stamp(d,base),*quaternion(d.orientation),d.angular_velocity.x,d.angular_velocity.y,d.angular_velocity.z,
                     d.linear_acceleration.x,d.linear_acceleration.y,d.linear_acceleration.z]
            elif name=='drive':row=[t,stamp(d,base),d.drive.steering_angle,d.drive.speed,d.drive.acceleration]
            elif name=='servo':row=[t,d.data]
            elif name=='core':row=[t,stamp(d,base),d.state.speed]
            elif name in ('odom','wheel'):
                pp=d.pose.pose;v=d.twist.twist
                row=[t,stamp(d,base),pp.position.x,pp.position.y,*quaternion(pp.orientation),v.linear.x,v.linear.y,v.angular.z]
            elif name=='pf':
                pp=d.pose.pose
                row=[t,stamp(d,base),pp.position.x,pp.position.y,*quaternion(pp.orientation)]
            values[name].append(row)
            if hasattr(d,'header'):frames[ch.topic]=d.header.frame_id
        out.mkdir(parents=True,exist_ok=True)
        name=path.parent.name+'_'+path.stem
        target=out/(name+'.npz')
        np.savez_compressed(target,**{k:np.asarray(v,dtype=float) for k,v in values.items()})
        metadata={'mcap':str(path),'size_bytes':path.stat().st_size,'epoch_base_s':int(base),
                  'frames':frames,'counts':{k:len(v) for k,v in values.items()},
                  'time_columns':'0=bag receipt time, 1=message stamp except servo; times relative to epoch_base_s'}
        target.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
        print(name,metadata['counts'],flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('bags',type=Path);p.add_argument('-o','--output',type=Path,required=True)
    a=p.parse_args()
    for bag in sorted(a.bags.rglob('*.mcap')):extract(bag,a.output)
