#!/usr/bin/env python3
"""
visualize.py

Fixed and improved launcher for:
 - ros_gz_bridge parameter_bridge (LaserScan + PointCloud2)
 - static_transform_publisher for the LiDAR frame
 - rviz2 launched with a generated RViz config

Added:
 - argparse options:
    -d / --drone   : drone name (default: leader_1)
    -w / --world   : gazebo world name (default: default)
    -v / --visual  : 0=no rviz, 1=yes rviz (default: 0)

 - RViz PointCloud display configured to use Style: "Points"
"""

from pathlib import Path
import subprocess
import threading
import time
import signal
import sys
import os
import shutil
import argparse

RVIZ_CFG_PATH = Path("/tmp/visualize_pointcloud.rviz")


def parse_args():
    p = argparse.ArgumentParser(description="Start ros_gz_bridge, static TF, and optional rviz for LiDAR visualization.")
    p.add_argument(
        "-d", "--drone",
        type=str,
        default="leader_1",
        help="Drone model name that contains the lidar sensor (e.g. leader_1 or x500_0). Default: 'leader_1'."
    )
    p.add_argument(
        "-w", "--world",
        type=str,
        default="default",
        help="Gazebo world name (PX4_GZ_WORLD). Default: 'default'."
    )
    p.add_argument(
        "-v", "--visual",
        type=int,
        choices=(0, 1),
        default=0,
        help="Whether to open rviz (1) or not (0). Default: 0."
    )
    return p.parse_args()


def check_executable(name):
    if shutil.which(name) is None:
        print(f"WARNING: executable '{name}' not found in PATH. Make sure it's installed and sourced.")


def make_rviz_config(path: Path, gz_pointcloud_topic: str, ros_pointcloud_topic: str):
    
    cfg = f"""Visualization Manager:
  Class: ''
  Displays:
    - Name: "PointCloud (bridged)"
      Class: "rviz_default_plugins/PointCloud2"
      Enabled: true
      Topic: "{gz_pointcloud_topic}"
      Queue Size: 10
      Style: "Points"

    - Name: "PointCloud (mapped)"
      Class: "rviz_default_plugins/PointCloud2"
      Enabled: true
      Topic: "{ros_pointcloud_topic}"
      Queue Size: 10
      Style: "Points"

  Global Options:
    Fixed Frame: map
"""
    path.write_text(cfg)
    print(f"Wrote RViz config to: {path}")


def start_process(cmd, name, pipe=True, env=None):
    """Start subprocess. If pipe=True, capture stdout/stderr and stream them."""
    print("Starting:", " ".join(cmd))
    if env is None:
        env = os.environ.copy()
    if pipe:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        # start streaming
        def printer(stream, tag):
            for line in iter(stream.readline, ""):
                if not line:
                    break
                print(f"[{name}:{tag}] {line.rstrip()}")
        threading.Thread(target=printer, args=(p.stdout, 'OUT'), daemon=True).start()
        threading.Thread(target=printer, args=(p.stderr, 'ERR'), daemon=True).start()
    else:
        # let process inherit parent's stdio so GUI apps get proper focus
        p = subprocess.Popen(cmd, env=env)
    return p


def shutdown_procs(procs):
    for p, name in procs:
        try:
            if p.poll() is None:
                print(f"Terminating {name} (pid {p.pid})")
                p.terminate()
                try:
                    p.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    print(f"Killing {name} (pid {p.pid})")
                    p.kill()
        except Exception as e:
            print(f"Error shutting down {name}: {e}")


def main():
    args = parse_args()
    drone = args.drone
    world = args.world
    visual = bool(args.visual)

    gz_laser_scan_topic = f"/world/{world}/model/{drone}/link/link/sensor/lidar_2d_v2/scan"
    gz_pointcloud_topic = f"/world/{world}/model/{drone}/link/link/sensor/lidar_2d_v2/scan/points"

    # ROS-side topics 
    ros_laser_topic = "/lidar/scan"
    ros_pointcloud_topic = "/lidar/pointcloud"

    lidar_frame = f"{drone}/link/lidar_2d_v2"
    static_tf_args = ["0", "0", "0", "0", "0", "0", "map", lidar_frame]

    # quick checks
    check_executable('ros2')
    check_executable('ros_gz_bridge')
    if visual:
        check_executable('rviz2')

    make_rviz_config(RVIZ_CFG_PATH, gz_pointcloud_topic, ros_pointcloud_topic)

    procs = []

    def handle_sigint(sig, frame):
        print('SIGINT received — shutting down')
        shutdown_procs(procs)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        # Build bridge command: map the Gazebo topics into ROS messages.
        bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{gz_laser_scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{gz_pointcloud_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # attempt CLI remaps for convenience (may be ignored on some versions)
            f"{gz_pointcloud_topic}:={ros_pointcloud_topic}",
            f"{gz_laser_scan_topic}:={ros_laser_topic}",
        ]
        p_bridge = start_process(bridge_cmd, 'bridge', pipe=True)
        procs.append((p_bridge, 'bridge'))

        time.sleep(0.5)

        tf_cmd = ["ros2", "run", "tf2_ros", "static_transform_publisher"] + static_tf_args
        p_tf = start_process(tf_cmd, 'static_tf', pipe=True)
        procs.append((p_tf, 'static_tf'))

        time.sleep(0.2)

        if visual:
            # start rviz without piping its stdio to avoid focus/input problems
            env = os.environ.copy()
            # set xcb to avoid Wayland pointer grab issues; if this breaks your setup remove the line
            env.setdefault('QT_QPA_PLATFORM', 'xcb')
            rviz_cmd = ["rviz2", "-d", str(RVIZ_CFG_PATH)]
            p_rviz = start_process(rviz_cmd, 'rviz', pipe=False, env=env)
            procs.append((p_rviz, 'rviz'))
            print("RViz launched.")
        else:
            print("RViz launch skipped (visual=0).")

        print("All processes started — press Ctrl+C to stop.")

        # monitor
        while True:
            time.sleep(1.0)
            for p, name in procs:
                if p.poll() is not None:
                    print(f"Process '{name}' exited with code {p.returncode}")
                    raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        shutdown_procs(procs)
        print("Done.")


if __name__ == '__main__':
    main()
