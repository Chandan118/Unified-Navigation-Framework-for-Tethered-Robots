#!/usr/bin/env python3
import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Train obstacle detector for robot camera.")
    parser.add_argument("--data", required=True, help="Path to YOLO dataset yaml.")
    parser.add_argument("--model", default="yolov8n.pt", help="Base model path.")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size.")
    parser.add_argument("--epochs", type=int, default=60, help="Training epochs.")
    parser.add_argument("--batch", type=int, default=16, help="Batch size.")
    parser.add_argument("--device", default="0", help="CUDA device id or 'cpu'.")
    parser.add_argument("--name", default="robot_obstacle_v1", help="Run name.")
    parser.add_argument("--project", default="/home/jetson/exp1_logs", help="Output folder.")
    args = parser.parse_args()

    data_path = Path(args.data)
    if not data_path.exists():
        raise FileNotFoundError(f"Dataset config not found: {data_path}")

    model = YOLO(args.model)
    model.train(
        data=str(data_path),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        name=args.name,
        project=args.project,
    )


if __name__ == "__main__":
    main()
