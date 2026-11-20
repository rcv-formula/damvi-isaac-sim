"""Request a trace snapshot without periodic serialization during simulation."""
import json
import subprocess
import sys


def flush_trace(root):
    result=subprocess.run([sys.executable,str(root.parent/'isaac_live_client.py'),
                           str(root/'tools/flush_gui_diagnostics.py')],
                          check=True,capture_output=True,text=True,timeout=15)
    response=json.loads(result.stdout)
    if response.get('status')!='ok':
        raise RuntimeError(response)
