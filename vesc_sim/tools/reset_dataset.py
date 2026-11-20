"""Reset one headless Isaac session to a dataset's map, bag friction and CSV pose."""
import argparse
import csv
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys
import uuid


def csv_pose(path, index, offset):
    with path.open(newline='') as stream:
        rows = [tuple(map(float,row[:2])) for row in csv.reader(stream) if row]
    if len(rows)<2 or not 0<=index<len(rows):
        raise ValueError('Invalid raceline or start index')
    x,y=rows[index]; nx,ny=rows[(index+1)%len(rows)]
    if math.hypot(nx-x,ny-y)<1e-6:
        raise ValueError('Degenerate raceline tangent')
    yaw=math.atan2(ny-y,nx-x)
    return [x-offset*math.sin(yaw), y+offset*math.cos(yaw), yaw]


REMOTE = '''
import json, socket, sys, time
from pathlib import Path
folder=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim'
request=json.loads(sys.argv[1])
script=(folder/'tools/reset_headless_dataset.py').read_text()
source='reset_request='+repr(request)+'\\n'+script
with socket.create_connection(('127.0.0.1',8226),timeout=10) as con:
    con.settimeout(60)
    con.sendall(json.dumps(dict(code=source, context='rc_car_setup', timeout=60)).encode())
    con.shutdown(socket.SHUT_WR)
    response=json.loads(b''.join(iter(lambda:con.recv(65536),b'')))
if response.get('status')!='ok':
    raise RuntimeError(response)
status=folder/'outputs/live_gui/dataset_reset.json'
deadline=time.monotonic()+60
while time.monotonic()<deadline:
    if status.exists():
        result=json.loads(status.read_text())
        if result.get('request_id')==request['request_id'] and result['status']!='running':
            print(json.dumps(result,indent=2));sys.exit(0 if result['status']=='ready' else 1)
    time.sleep(.2)
raise RuntimeError('Timed out waiting for dataset reset')
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset')
    parser.add_argument('--dataset-root',type=Path,default=Path.home()/'Desktop/red-damvi dataset')
    parser.add_argument('--csv-name',default='latest.csv')
    parser.add_argument('--index',type=int,default=0)
    parser.add_argument('--offset',type=float,default=0,help='Signed left offset from the selected CSV point (m)')
    parser.add_argument('--local-isaac',action='store_true',help='Run on the Isaac host with its local Kit socket')
    parser.add_argument('--host',default='shin@192.168.0.2')
    parser.add_argument('--socket',help='Optional existing OpenSSH control socket')
    args=parser.parse_args()
    spawn=csv_pose(args.dataset_root/args.dataset/args.csv_name,args.index,args.offset)
    request={'request_id':str(uuid.uuid4()),'dataset':args.dataset,'spawn':spawn}
    cmd=['ssh']+(['-S',args.socket] if args.socket else [])+[args.host, 'python3 - '+shlex.quote(json.dumps(request))]
    if args.local_isaac:
        cmd=[sys.executable, '-', json.dumps(request)]
    return subprocess.run(cmd,input=REMOTE,text=True).returncode


if __name__=='__main__':
    sys.exit(main())
