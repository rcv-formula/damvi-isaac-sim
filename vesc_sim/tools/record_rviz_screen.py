#!/usr/bin/env python3
"""Capture an existing X11 window on an optional shared UTC epoch.

The X11 window ID capture works with this desktop compositor even while another
window covers RViz. It does not move, raise, resize, or focus any window. The
sidecar records the X11 source's first frame UTC timestamp for later alignment
with a camera recording on another machine (whose clock must also be checked).
"""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time


def utc_iso(epoch):
    return dt.datetime.fromtimestamp(epoch, dt.timezone.utc).isoformat()


def run_text(command, env):
    return subprocess.check_output(command, env=env, text=True, stderr=subprocess.STDOUT)


def identify_window(args, env):
    if args.window_id:
        window_id = int(args.window_id, 0)
    else:
        tree = run_text(["xwininfo", "-root", "-tree"], env)
        matching = []
        for line in tree.splitlines():
            if args.window_title in line:
                match = re.match(r'\s*(0x[0-9a-fA-F]+)\s+"', line)
                if match:
                    matching.append(int(match.group(1), 16))
        if len(matching) != 1:
            raise RuntimeError(
                f"Expected one window matching {args.window_title!r}; found {matching}. "
                "Specify --window-id from xwininfo -root -tree."
            )
        window_id = matching[0]
    info = run_text(["xwininfo", "-id", hex(window_id)], env)
    def field(name):
        return int(re.search(rf"^\s*{re.escape(name)}:\s*(-?\d+)", info, re.M).group(1))
    if "Map State: IsViewable" not in info:
        raise RuntimeError("The selected RViz window is not mapped/viewable.")
    return {
        "id": window_id,
        "id_hex": hex(window_id),
        "width": field("Width"),
        "height": field("Height"),
        "screen_x": field("Absolute upper-left X"),
        "screen_y": field("Absolute upper-left Y"),
        "xwininfo": info,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output MP4, with JSON and log sidecars")
    parser.add_argument("--start-epoch", type=float,
                        help="UTC Unix epoch shared with the other recording; defaults to now")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0"))
    parser.add_argument("--window-id", help="Existing main-window ID, decimal or 0x hex")
    parser.add_argument("--window-title", default=" - RViz")
    parser.add_argument("--crop", type=int, nargs=4, metavar=("X", "Y", "WIDTH", "HEIGHT"),
                        help="Optional crop in client-window coordinates, before scaling")
    parser.add_argument("--scale", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"),
                        help="Optional output size after cropping")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.fps <= 0 or args.threads <= 0:
        parser.error("duration, fps, and threads must be positive")
    if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        parser.error("ffmpeg and ffprobe are required")
    output = args.output.expanduser().resolve()
    if output.exists() and not args.overwrite:
        parser.error(f"Output already exists: {output}; use --overwrite explicitly")
    output.parent.mkdir(parents=True, exist_ok=True)
    sidecar = output.with_suffix(".timestamps.json")
    log_path = output.with_suffix(".ffmpeg.log")
    env = {**os.environ, "DISPLAY": args.display}
    window = identify_window(args, env)
    filters = []
    if args.crop:
        crop_x, crop_y, crop_width, crop_height = args.crop
        if (crop_x < 0 or crop_y < 0 or crop_width <= 0 or crop_height <= 0
                or crop_x + crop_width > window["width"]
                or crop_y + crop_height > window["height"]):
            parser.error("Crop must lie inside the selected client window")
        filters.append(f"crop={crop_width}:{crop_height}:{crop_x}:{crop_y}")
    if args.scale:
        scale_width, scale_height = args.scale
        if scale_width <= 0 or scale_height <= 0:
            parser.error("Scale width and height must be positive")
        filters.append(f"scale={scale_width}:{scale_height}:flags=lanczos")
    filters.append("pad=ceil(iw/2)*2:ceil(ih/2)*2")
    now = time.time()
    scheduled = args.start_epoch if args.start_epoch is not None else now
    if scheduled < now - 0.5:
        parser.error(f"Scheduled start is {now - scheduled:.3f} seconds in the past")
    command = [
        "ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "info",
        "-y" if args.overwrite else "-n", "-f", "x11grab",
        "-window_id", str(window["id"]), "-draw_mouse", "0",
        "-framerate", str(args.fps),
        "-video_size", f'{window["width"]}x{window["height"]}',
        "-i", args.display, "-t", str(args.duration), "-an",
        "-vf", ",".join(filters), "-vsync", "cfr",
        "-c:v", "libx264", "-threads", str(args.threads),
        "-preset", "veryfast", "-crf", str(args.crf),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        "-metadata", f"creation_time={utc_iso(scheduled)}", str(output),
    ]
    metadata = {
        "status": "armed", "output": str(output), "window": window,
        "display": args.display, "scheduled_start_epoch": scheduled,
        "scheduled_start_utc": utc_iso(scheduled),
        "requested_duration_s": args.duration, "requested_fps": args.fps,
        "capture": "actual X11 client window via x11grab window_id",
        "crop_xywh": args.crop, "scale_wh": args.scale,
        "video_filter": ",".join(filters),
        "command": command, "ffmpeg_log": str(log_path),
        "alignment_note": "Use first_source_frame_epoch, after compensating inter-host clock offset; no hardware synchronization is claimed.",
    }

    def write_metadata():
        temp = sidecar.with_suffix(sidecar.suffix + ".tmp")
        temp.write_text(json.dumps(metadata, indent=2) + "\n")
        temp.replace(sidecar)

    write_metadata()
    print(json.dumps({"status": "armed", "scheduled_start_epoch": scheduled,
                      "window": window["id_hex"], "output": str(output)}), flush=True)
    deadline = time.monotonic() + max(0.0, scheduled - time.time())
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(remaining, 0.25))
    metadata["launch_epoch"] = time.time()
    metadata["launch_utc"] = utc_iso(metadata["launch_epoch"])
    metadata["launch_delay_s"] = metadata["launch_epoch"] - scheduled
    metadata["status"] = "recording"
    write_metadata()
    with log_path.open("w") as log:
        process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=log)
        metadata["ffmpeg_pid"] = process.pid
        write_metadata()
        try:
            returncode = process.wait(timeout=args.duration + 45)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            process.terminate()
            try:
                returncode = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                returncode = process.wait()
    metadata["completed_epoch"] = time.time()
    metadata["completed_utc"] = utc_iso(metadata["completed_epoch"])
    metadata["returncode"] = returncode
    match = re.search(r"Duration: N/A, start: ([0-9.]+)", log_path.read_text())
    if match:
        source_start = float(match.group(1))
        metadata["first_source_frame_epoch"] = source_start
        metadata["first_source_frame_utc"] = utc_iso(source_start)
        metadata["source_start_delay_s"] = source_start - scheduled
    if returncode == 0:
        metadata["media"] = json.loads(run_text([
            "ffprobe", "-v", "error", "-show_entries",
            "format=duration,size:stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_frames,duration",
            "-of", "json", str(output),
        ], env))
        metadata["status"] = "complete"
    else:
        metadata["status"] = "failed"
    write_metadata()
    print(json.dumps({"status": metadata["status"], "timestamps": str(sidecar),
                      "first_source_frame_epoch": metadata.get("first_source_frame_epoch"),
                      "returncode": returncode}), flush=True)
    return 0 if returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
