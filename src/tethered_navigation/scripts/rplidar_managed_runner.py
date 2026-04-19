#!/usr/bin/env python3
import argparse
import signal
import subprocess
import sys
import time

import serial


_stop_requested = False


def _on_signal(signum, frame):
    del signum, frame
    global _stop_requested
    _stop_requested = True


def _reset_lidar(port: str, baud: int) -> None:
    try:
        lidar = serial.Serial(port, baudrate=baud, timeout=0.5)
        lidar.write(b"\xA5\x25")  # STOP
        time.sleep(0.05)
        lidar.write(b"\xA5\x40")  # RESET
        time.sleep(0.20)
        _ = lidar.read(64)
        lidar.close()
        print(f"[rplidar_managed_runner] reset sent on {port} @ {baud}", flush=True)
    except Exception as exc:
        print(f"[rplidar_managed_runner] reset skipped: {exc}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Managed RPLidar launcher with serial reset.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--frame-id", default="front_lidar_link")
    parser.add_argument("--scan-mode", default="Sensitivity")
    parser.add_argument("--restart-delay", type=float, default=1.5)
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    while not _stop_requested:
        _reset_lidar(args.port, args.baud)
        cmd = [
            "ros2",
            "run",
            "rplidar_ros",
            "rplidar_node",
            "--ros-args",
            "-p",
            f"serial_port:={args.port}",
            "-p",
            f"serial_baudrate:={args.baud}",
            "-p",
            f"frame_id:={args.frame_id}",
            "-p",
            f"scan_mode:={args.scan_mode}",
        ]
        print(f"[rplidar_managed_runner] starting: {' '.join(cmd)}", flush=True)
        proc = subprocess.Popen(cmd)
        while proc.poll() is None and not _stop_requested:
            time.sleep(0.2)
        if _stop_requested:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
            break
        print(
            f"[rplidar_managed_runner] rplidar_node exited with {proc.returncode}, restarting...",
            flush=True,
        )
        time.sleep(args.restart_delay)

    print("[rplidar_managed_runner] stopped", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
