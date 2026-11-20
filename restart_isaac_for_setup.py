"""Restart the identified blank streaming app with localhost Python Server."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

pid = 6558
proc = Path('/proc') / str(pid)
args = [part.decode() for part in (proc / 'cmdline').read_bytes().split(b'\0') if part]
expected = ['/home/shin/isaac_sim/kit/kit', '/home/shin/isaac_sim/apps/isaacsim.exp.full.streaming.kit', '--no-window']
if args != expected:
    raise RuntimeError('Current process arguments differ from the inspected streaming app; refusing restart.')
env = dict(part.decode().split('=', 1) for part in (proc / 'environ').read_bytes().split(b'\0') if b'=' in part)
cwd = os.readlink(proc / 'cwd')
output = Path('/home/shin/Desktop/RC_Car_Isaac')
log = output / 'setup_streaming.log'
if log.exists():
    raise FileExistsError(log)
new_args = args + ['--enable', 'isaacsim.code_editor.python_server']
print(json.dumps({'original_pid': pid, 'arguments': new_args, 'cwd': cwd, 'log': str(log)}), flush=True)
os.kill(pid, signal.SIGINT)
for _ in range(300):
    if not proc.exists():
        break
    time.sleep(0.1)
else:
    raise RuntimeError('Original app did not exit after SIGINT; no force termination attempted.')
with log.open('xb') as stream:
    child = subprocess.Popen(new_args, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
print(json.dumps({'new_pid': child.pid}), flush=True)
