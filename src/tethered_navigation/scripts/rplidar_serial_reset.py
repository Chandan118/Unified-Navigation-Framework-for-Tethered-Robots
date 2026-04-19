#!/usr/bin/env python3
import argparse
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Send stop+reset to RPLidar over serial.")
    parser.add_argument("--port", required=True, help="Serial port path, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--timeout", type=float, default=0.5, help="Serial timeout seconds")
    args = parser.parse_args()

    # Best-effort reset: this should not crash launch if reset fails.
    try:
        lidar = serial.Serial(args.port, baudrate=args.baud, timeout=args.timeout)
        lidar.write(b"\xA5\x25")  # STOP
        time.sleep(0.05)
        lidar.write(b"\xA5\x40")  # RESET
        time.sleep(0.20)
        _ = lidar.read(64)
        lidar.close()
        print(f"RPLidar serial reset sent on {args.port} @ {args.baud}")
    except Exception as exc:
        print(f"RPLidar serial reset skipped ({args.port}): {exc}")


if __name__ == "__main__":
    main()
