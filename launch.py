#!/usr/bin/env python3

import os
import sys
import time
import subprocess
import json
import argparse
from pathlib import Path

def parse_args():
    default_points = Path(__file__).parent / "points" / "drone_points.json"
    p = argparse.ArgumentParser(description="Launch multiple PX4 SITL instances via tmux.")
    p.add_argument(
        "-p", "--points",
        type=str,
        default=str(default_points),
        help="Path to drone_points.json file or directory containing it. "
             f"Default: {default_points}"
    )
    p.add_argument(
        "-w", "--world",
        type=str,
        default="default",
        help="Name of the Gazebo world to use (PX4_GZ_WORLD). Default: 'default'."
    )
    return p.parse_args()

def resolve_points_path(points_arg: str) -> Path:
    p = Path(points_arg).expanduser()
    if p.is_dir():
        p = p / "drone_points.json"
    return p

def main():
    args = parse_args()
    world = args.world
    points_path = resolve_points_path(args.points)

    if not points_path.exists():
        print(f"File not found: {points_path}")
        return

    try:
        with points_path.open("r") as f:
            drone_targets = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Failed to parse JSON ({points_path}): {e}")
        return

    # number of drones is inferred from the first saved drawing (key "0")
    if "0" not in drone_targets:
        print("JSON does not contain key '0' with drone points. Check the file format.")
        return

    num_drones = len(drone_targets["0"])
    session = "px4_multi"

    # Start new tmux session (detached)
    subprocess.run(["tmux", "new-session", "-d", "-s", session])

    for i in range(1, num_drones + 2):
        window_name = f"Drone-{i}"
        window_index = i - 1
        pose = i

        # Command to run inside tmux window
        if i == 1:
            # Rename initial window
            subprocess.run(["tmux", "rename-window", "-t", f"{session}:0", window_name])
            cmd = (
                f"cd ~/PX4-Autopilot && "
                f"PX4_SYS_AUTOSTART=4001 "
                f"PX4_GZ_MODEL_POSE=\"0,{pose}\" "
                f"PX4_GZ_WORLD={world} "
                f"PX4_SIM_MODEL=gz_leader "
                f"./build/px4_sitl_default/bin/px4 -i 1"
            )
        else:
            # Create new window
            subprocess.run(["tmux", "new-window", "-t", session, "-n", window_name])
            cmd = (
                f"cd ~/PX4-Autopilot && "
                f"PX4_GZ_STANDALONE=1 "
                f"PX4_SYS_AUTOSTART=4001 "
                f"PX4_GZ_MODEL_POSE=\"0,{pose}\" "
                f"PX4_GZ_WORLD={world} "
                f"PX4_SIM_MODEL=gz_x500 "
                f"./build/px4_sitl_default/bin/px4 -i {i}"
            )

        # Send the command to tmux window
        subprocess.run(["tmux", "send-keys", "-t", f"{session}:{window_index}", cmd, "C-m"])

        # Wait before launching next drone
        time.sleep(7)

    # Launching Ground Control
    subprocess.run([
        "tmux", "new-window", "-t", session, "-n", "GroundControl",
        "bash", "-c", "cd ~ && ./QGroundControl-x86_64.AppImage"
    ])
    time.sleep(2)

    # Launching Micro XRCE Agent for ROS2 Topics
    subprocess.run([
        "tmux", "new-window", "-t", session, "-n", "Micro XRCE Agent",
        "bash", "-c", "MicroXRCEAgent udp4 -p 8888"
    ])
    time.sleep(2)

    # Attach to the tmux session
    subprocess.run(["tmux", "attach-session", "-t", session])

if __name__ == "__main__":
    main()
