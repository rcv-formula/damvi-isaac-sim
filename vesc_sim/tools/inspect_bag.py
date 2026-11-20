"""Read selected MCAP topics without replaying anything onto a ROS network.

pip install mcap-ros2-support. Accepts an MCAP or a directory of ROS bags.
"""
import argparse
from bisect import bisect_left
import hashlib
import json
from pathlib import Path
import statistics
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def inspect(path):
    topics={}
    with path.open('rb') as f:
        reader=make_reader(f,decoder_factories=[DecoderFactory()])
        for schema,ch,msg,value in reader.iter_decoded_messages(topics=[
                '/commands/motor/speed','/commands/motor/brake','/sensors/core','/drive','/odom_wheel']):
            if ch.topic=='/sensors/core':number=value.state.speed
            elif ch.topic=='/drive':number=value.drive.speed
            elif ch.topic=='/odom_wheel':number=value.twist.twist.linear.x
            else:number=value.data
            topics.setdefault(ch.topic,[]).append((msg.log_time,float(number)))
    out={'mcap':str(path),'topics':{}}
    for topic,samples in topics.items():
        values=[v for _,v in samples]
        intervals=[(b[0]-a[0])*1e-9 for a,b in zip(samples,samples[1:]) if b[0]>a[0]]
        out['topics'][topic]={'count':len(samples),'min':min(values),'max':max(values),
            'median_rate_hz':1/statistics.median(intervals) if intervals else None,
            'positive':sum(v>0 for v in values),'negative':sum(v<0 for v in values)}
    commands=topics.get('/commands/motor/speed',[]);drive=topics.get('/drive',[])
    if commands and drive:
        stamps=[t for t,_ in drive];ratios=[]
        for t,v in commands:
            idx=min(bisect_left(stamps,t),len(drive)-1)
            if idx and abs(stamps[idx-1]-t)<abs(stamps[idx]-t):idx-=1
            td,vd=drive[idx]
            if abs(td-t)<20_000_000 and abs(vd)>.2:ratios.append(v/vd)
        if ratios:out['recorded_command_gain_median']=statistics.median(ratios)
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);p.add_argument('-o','--output',type=Path,required=True)
    a=p.parse_args();files=[a.path] if a.path.is_file() else sorted(a.path.rglob('*.mcap'))
    result=[inspect(f) for f in files];a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False));print(a.output)
