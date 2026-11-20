"""Execute a trusted local Python file in Isaac Sim's localhost Python Server."""
import argparse
import json
import socket
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("script", type=Path)
parser.add_argument("--timeout", type=float, default=60)
parser.add_argument("--port", type=int, default=8226)
args = parser.parse_args()
envelope = {
    "code": args.script.read_text(encoding="utf-8-sig"),
    "context": "rc_car_setup",
    "timeout": args.timeout,
}
with socket.create_connection(("127.0.0.1", args.port), timeout=10) as connection:
    connection.settimeout(args.timeout + 15)
    connection.sendall(json.dumps(envelope).encode())
    connection.shutdown(socket.SHUT_WR)
    chunks = []
    while chunk := connection.recv(65536):
        chunks.append(chunk)
result = json.loads(b"".join(chunks).decode())
print(json.dumps(result, ensure_ascii=False, indent=2))
raise SystemExit(0 if result.get("status") == "ok" else 1)
