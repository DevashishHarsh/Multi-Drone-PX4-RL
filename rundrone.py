#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
from sshkeyboard import listen_keyboard, stop_listening
import threading
import time
import json
import os
import numpy as np
import pickle
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize
import traceback
import math
import argparse
from pathlib import Path

# ----------------- Config -----------------
LIDAR_TOPIC = "/lidar/scan"
LIDAR_NUM_RAYS = 60
LIDAR_MAX_RANGE = 10.0
LIDAR_INDEX_SHIFT = 30

MAX_VEL = 2.0
GOAL_THRESHOLD = 1.0
MODEL_DEVICE = "cpu"   # default; overridden by CLI -d/--device
DEBUG_PRINT = True
DEBUG_PRINT_FREQ = 10
# ------------------------------------------

class OffboardControl(Node):
    def __init__(self, drone_id=1):
        super().__init__(f'offboard_cli_control_{drone_id}')
        self.drone_id = drone_id
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        prefix = f'/px4_{drone_id}/fmu'
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', qos_pub)
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f'{prefix}/out/vehicle_local_position', self.local_position_cb, qos_sub)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, f'{prefix}/out/vehicle_status', self.status_cb, qos_sub)
        self.lidar_sub = self.create_subscription(LaserScan, LIDAR_TOPIC, self._lidar_cb, qos_sub)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_started = False
        self.current_setpoint = [0.0, 0.0, 0.0]
        self.base_setpoint = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.yaw_target = None
        self.yaw_kp = 1.2
        self.yaw_ki = 0.01
        self.yaw_kd = 0.02
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.yaw_integral_limit = 1.0
        self.timer_period = 0.1
        self.offboard_counter = 0
        self.flying_mode = 0

        self.lidar_data = None
        self.lidar_raw = None
        self.lidar_shifted_raw = None
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.lidar_computed_shift = None

        self.timer = self.create_timer(self.timer_period, self.timer_cb)

    def local_position_cb(self, msg: VehicleLocalPosition):
        try:
            self.vehicle_local_position = msg
            if hasattr(msg, 'yaw'):
                try:
                    self.current_yaw = float(msg.yaw)
                except Exception:
                    pass
            elif hasattr(msg, 'yaw_computed'):
                try:
                    self.current_yaw = float(msg.yaw_computed)
                except Exception:
                    pass
        except Exception as e:
            print("[LP CB] Error storing local position:", e)

    def status_cb(self, msg: VehicleStatus):
        try:
            self.vehicle_status = msg
        except Exception as e:
            print("[STATUS CB] Error storing vehicle status:", e)

    def publish_offboard_control_mode(self):
        try:
            msg = OffboardControlMode()
            msg.position = True
            msg.velocity = True
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            self.offboard_control_mode_pub.publish(msg)
        except Exception as e:
            print("[TIMER] publish_offboard_control_mode error:", e)

    def publish_trajectory_setpoint(self, x, y, z, vx, vy, vz, yaw):
        try:
            msg = TrajectorySetpoint()
            px = float(x) if (x is not None and not (isinstance(x, float) and math.isnan(x))) else float('nan')
            py = float(y) if (y is not None and not (isinstance(y, float) and math.isnan(y))) else float('nan')
            pz = float(z) if (z is not None and not (isinstance(z, float) and math.isnan(z))) else float('nan')
            vx_f = float(vx) if (vx is not None and not (isinstance(vx, float) and math.isnan(vx))) else float('nan')
            vy_f = float(vy) if (vy is not None and not (isinstance(vy, float) and math.isnan(vy))) else float('nan')
            vz_f = float(vz) if (vz is not None and not (isinstance(vz, float) and math.isnan(vz))) else float('nan')
            try:
                msg.position = [px, py, pz]
                msg.velocity = [vx_f, vy_f, vz_f]
            except Exception:
                try:
                    msg.position[0] = px
                    msg.position[1] = py
                    msg.position[2] = pz
                    msg.velocity[0] = vx_f
                    msg.velocity[1] = vy_f
                    msg.velocity[2] = vz_f
                except Exception:
                    print("[TIMER] Warning: couldn't assign position/velocity arrays directly.")
            try:
                msg.yaw = float(yaw) if (yaw is not None and not (isinstance(yaw, float) and math.isnan(yaw))) else float('nan')
            except Exception:
                pass
            self.trajectory_setpoint_pub.publish(msg)
        except Exception as e:
            print("[TIMER] publish_trajectory_setpoint error:", e)

    def publish_vehicle_command(self, cmd, **kwargs):
        try:
            msg = VehicleCommand()
            msg.param1 = float(kwargs.get("param1", 0.0))
            msg.param2 = float(kwargs.get("param2", 0.0))
            msg.param3 = float(kwargs.get("param3", 0.0))
            msg.param4 = float(kwargs.get("param4", 0.0))
            msg.param5 = kwargs.get("param5", 0.0)
            msg.param6 = kwargs.get("param6", 0.0)
            msg.param7 = kwargs.get("param7", 0.0)
            msg.command = int(cmd)
            msg.target_system = self.drone_id + 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            self.vehicle_command_pub.publish(msg)
        except Exception as e:
            print("[CMD] publish_vehicle_command error:", e)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_cb(self):
        try:
            self.publish_offboard_control_mode()
            yaw_to_send = None
            if self.yaw_target is None:
                yaw_to_send = float(self.current_yaw)
                self.yaw_integral = 0.0
                self.yaw_prev_error = 0.0
            else:
                def wrap_to_pi(a):
                    return (a + math.pi) % (2 * math.pi) - math.pi
                error = wrap_to_pi(self.yaw_target - self.current_yaw)
                self.yaw_integral += error * self.timer_period
                if self.yaw_integral > self.yaw_integral_limit:
                    self.yaw_integral = self.yaw_integral_limit
                elif self.yaw_integral < -self.yaw_integral_limit:
                    self.yaw_integral = -self.yaw_integral_limit
                derivative = (error - self.yaw_prev_error) / self.timer_period
                pid_out = (self.yaw_kp * error) + (self.yaw_ki * self.yaw_integral) + (self.yaw_kd * derivative)
                max_step = 0.5
                if pid_out > max_step:
                    pid_out = max_step
                elif pid_out < -max_step:
                    pid_out = -max_step
                yaw_to_send = float(self.current_yaw + pid_out)
                self.yaw_prev_error = error

            self.publish_trajectory_setpoint(
                self.current_setpoint[0] if len(self.current_setpoint) > 0 else None,
                self.current_setpoint[1] if len(self.current_setpoint) > 1 else None,
                self.current_setpoint[2] if len(self.current_setpoint) > 2 else None,
                self.current_velocity[0] if len(self.current_velocity) > 0 else None,
                self.current_velocity[1] if len(self.current_velocity) > 1 else None,
                self.current_velocity[2] if len(self.current_velocity) > 2 else None,
                yaw_to_send
            )
            if not self.offboard_started:
                if self.offboard_counter == 10:
                    self.engage_offboard()
                    self.arm()
                    self.offboard_started = True
                else:
                    self.offboard_counter += 1
        except Exception as e:
            print("[TIMER] Exception in timer_cb:", e)

    _LP_ALPHA = 0.2
    def _lidar_cb(self, msg: LaserScan):
        try:
            try:
                self.lidar_angle_min = float(msg.angle_min)
                self.lidar_angle_increment = float(msg.angle_increment)
            except Exception:
                self.lidar_angle_min = None
                self.lidar_angle_increment = None

            new_ranges = np.array(msg.ranges, dtype=np.float32)
            inf_mask = np.isinf(new_ranges)
            if np.any(inf_mask):
                new_ranges[inf_mask] = LIDAR_MAX_RANGE

            if new_ranges.shape[0] < LIDAR_NUM_RAYS:
                padded = np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, dtype=np.float32)
                padded[:new_ranges.shape[0]] = new_ranges
                raw_padded = padded
            elif new_ranges.shape[0] > LIDAR_NUM_RAYS:
                raw_padded = new_ranges[:LIDAR_NUM_RAYS]
            else:
                raw_padded = new_ranges.copy()

            self.lidar_raw = raw_padded.copy()
            desired_shift = int(LIDAR_INDEX_SHIFT)
            self.lidar_computed_shift = desired_shift
            rotated = np.roll(raw_padded, -desired_shift)
            self.lidar_shifted_raw = rotated.copy()
            if isinstance(self.lidar_data, np.ndarray) and self.lidar_data.shape[0] == LIDAR_NUM_RAYS:
                self.lidar_data = self._LP_ALPHA * rotated + (1.0 - self._LP_ALPHA) * self.lidar_data
            else:
                self.lidar_data = rotated.copy()
        except Exception as e:
            self.lidar_data = np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, dtype=np.float32)
            self.lidar_raw = self.lidar_data.copy()
            self.lidar_shifted_raw = self.lidar_data.copy()
            self.lidar_computed_shift = None
            print("[LIDAR] Failed to process LaserScan:", e)

def multi_drone_move(targets: list, model_dir: str = None, device: str = "cpu"):
    """
    Run multi-drone formation control. model_dir should be a path to a folder
    that contains model.zip and optionally model_vector.pkl. device is 'cpu' or 'cuda'.
    """
    rclpy.init()
    drone_nodes = {}
    drone_number = len(targets[0])
    movement = {"x": 0.0, "y": 0.0, "z": 0.0}
    yaw = 0.0
    manual_running = False
    leader_id = 1
    current_set_index = 0

    auto_running = False
    auto_model = None
    auto_vecnorm = None
    prev_action = np.array([0.0, 0.0], dtype=np.float32)
    action_smooth_alpha = 0.4
    lidar_data = None
    goal_pos = None
    leader_z_locked = None
    model_loaded = False
    at_goal = False

    keyboard_thread = None
    keyboard_thread_started = False

    def keyboard_listener_thread():
        listen_keyboard(on_press=press, on_release=release, sequential=False)

    def compute_formation(set_index):
        target = targets[set_index]
        x_vals = [float(pos[0]) for k, pos in target.items()]
        y_vals = [float(pos[1]) for k, pos in target.items()]
        center_of_mass = [float(np.mean(x_vals)) - leader_id, float(np.mean(y_vals)), -10.0]
        offsets = [None] * (drone_number + 2)
        for drone_id_str, position in target.items():
            drone_id = int(drone_id_str)
            world_x = position[0] - drone_id
            offsets[drone_id] = [
                world_x - center_of_mass[0],
                position[1] - center_of_mass[1],
                position[2] - center_of_mass[2]
            ]
        offsets[leader_id] = [0.0, 0.0, 0.0]
        return center_of_mass, offsets

    center_of_mass, offsets = compute_formation(current_set_index)
    print(f"Initial center of mass: {center_of_mass}")
    print(f"Initial offsets: {offsets}")

    def run_node(drone_id, final_position, is_leader=False):
        node = OffboardControl(drone_id)
        drone_nodes[drone_id] = node
        node.base_setpoint = final_position.copy()
        def spin_node():
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            executor.spin()
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        time.sleep(2.0)
        takeoff_position = [0.0, 0.0, -10.0 if is_leader else -5.0]
        node.current_setpoint = takeoff_position
        print(f"Drone {drone_id}{' (Leader)' if is_leader else ''} taking off to {takeoff_position}")
        for _ in range(20):
            node.publish_offboard_control_mode()
            node.publish_trajectory_setpoint(*takeoff_position, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)
        node.engage_offboard()
        node.arm()
        time.sleep(2.0)
        node.current_setpoint = final_position.copy()
        print(f"Drone {drone_id}{' (Leader)' if is_leader else ''} at {final_position}")
        if not is_leader:
            for _ in range(20):
                node.publish_trajectory_setpoint(*final_position, 0.0, 0.0, 0.0, 0.0)
                time.sleep(0.1)

    def press(key):
        nonlocal movement, yaw, manual_running, current_set_index, center_of_mass, offsets
        nonlocal auto_running, keyboard_thread_started, keyboard_thread
        movement_step = 3.0
        yaw_step = 0.2
        if key == "w":
            movement["x"] = movement_step
        elif key == "s":
            movement["x"] = -movement_step
        elif key == "a":
            movement["y"] = movement_step
        elif key == "d":
            movement["y"] = -movement_step
        elif key == "i":
            movement["z"] = -movement_step
        elif key == "k":
            movement["z"] = movement_step
        elif key == "j":
            yaw = yaw_step
        elif key == "l":
            yaw = -yaw_step
        elif key == "n":
            if current_set_index + 1 < len(targets):
                current_set_index += 1
                print(f"Moving to point set {current_set_index}")
                center_of_mass, offsets = compute_formation(current_set_index)
                print(f"New center of mass: {center_of_mass}")
                print(f"New offsets: {offsets}")
                for drone_id, node in drone_nodes.items():
                    node.flying_mode = 0
                    node.current_velocity = [0.0, 0.0, 0.0]
                    node.current_yaw = 0.0
                    if drone_id == leader_id:
                        node.base_setpoint = center_of_mass.copy()
                        node.current_setpoint = center_of_mass.copy()
                    else:
                        world_x = targets[current_set_index][str(drone_id)][0] - drone_id
                        final_position = [world_x, targets[current_set_index][str(drone_id)][1], targets[current_set_index][str(drone_id)][2]]
                        node.base_setpoint = final_position.copy()
                        node.current_setpoint = final_position.copy()
                    print(f"Drone {drone_id} moving to new setpoint {node.current_setpoint}")
                stabilize_formation()
                print(f"Reached point set {current_set_index}. Resuming previous mode.")
                drone_nodes[leader_id].flying_mode = 1
                drone_nodes[leader_id].current_setpoint = [float('nan'), float('nan'), float('nan')]
                drone_nodes[leader_id].current_velocity = [0.0, 0.0, 0.0]
            else:
                print("No more point sets available.")
        elif key == "esc":
            if keyboard_thread_started:
                try:
                    stop_listening()
                except Exception:
                    pass
                keyboard_thread_started = False
                keyboard_thread = None
            manual_running = False
            for node in drone_nodes.values():
                node.flying_mode = 0
                node.current_velocity = [0.0, 0.0, 0.0]
                node.current_yaw = 0.0
            print("\nExited manual mode.")
        elif key == "z":
            if keyboard_thread_started and manual_running:
                try:
                    stop_listening()
                except Exception:
                    pass
                keyboard_thread_started = False
                keyboard_thread = None
                manual_running = False
                for node in drone_nodes.values():
                    node.flying_mode = 0
                    node.current_velocity = [0.0, 0.0, 0.0]
                    node.current_yaw = 0.0
                print("Exited MANUAL mode via 'z'. Returned to menu.")
            elif keyboard_thread_started and auto_running:
                auto_running = False
                print("Exiting AUTO mode via 'z' key (listener).")

    def release(key):
        nonlocal movement, yaw
        if key in ["w", "s"]:
            movement["x"] = 0.0
        elif key in ["a", "d"]:
            movement["y"] = 0.0
        elif key in ["i", "k"]:
            movement["z"] = 0.0
        elif key in ["j", "l"]:
            yaw = 0.0

    def is_valid_position(pos):
        return all(isinstance(v, float) and not np.isnan(v) and abs(v) < 1e6 for v in [pos.x, pos.y, pos.z])

    def stabilize_formation():
        print("Stabilizing formation...")
        start_time = time.time()
        while time.time() - start_time < 15.0:
            all_ready = True
            for drone_id, node in drone_nodes.items():
                if is_valid_position(node.vehicle_local_position):
                    error = [
                        node.current_setpoint[0] - node.vehicle_local_position.x,
                        node.current_setpoint[1] - node.vehicle_local_position.y,
                        node.current_setpoint[2] - node.vehicle_local_position.z
                    ]
                    if any(abs(e) > 0.5 for e in error):
                        all_ready = False
                        print(f"Drone {drone_id}: Error {error}, stabilizing...")
                else:
                    all_ready = False
                    print(f"Drone {drone_id}: Invalid position [{node.vehicle_local_position.x}, {node.vehicle_local_position.y}, {node.vehicle_local_position.z}]")
                node.publish_trajectory_setpoint(*node.current_setpoint, 0.0, 0.0, 0.0, 0.0)
            if all_ready:
                print("All drones reached new setpoints.")
                break
            time.sleep(0.1)

    def update_loop():
        kp = 0.5
        dt = 0.01
        while True:
            if not manual_running:
                time.sleep(0.01)
                continue
            leader_node = drone_nodes[leader_id]
            leader_node.flying_mode = 1
            leader_node.current_velocity = [movement["y"], movement["x"], movement["z"]]
            leader_node.current_setpoint = [float('nan'), float('nan'), float('nan')]
            leader_pos = [
                leader_node.vehicle_local_position.x,
                leader_node.vehicle_local_position.y,
                leader_node.vehicle_local_position.z
            ]
            for drone_id, node in drone_nodes.items():
                if drone_id != leader_id:
                    desired_pos = [
                        leader_pos[0] + offsets[drone_id][0],
                        leader_pos[1] + offsets[drone_id][1],
                        leader_pos[2] + offsets[drone_id][2]
                    ]
                    pos_error = [0.0, 0.0, 0.0]
                    if is_valid_position(node.vehicle_local_position):
                        pos_error = [
                            desired_pos[0] - node.vehicle_local_position.x,
                            desired_pos[1] - node.vehicle_local_position.y,
                            desired_pos[2] - node.vehicle_local_position.z
                        ]
                    node.flying_mode = 0
                    node.current_velocity = [kp * e for e in pos_error]
                    node.current_setpoint = desired_pos
                    node.current_yaw += yaw * dt
            time.sleep(dt)

    # ----- AUTO: integrate leader code here -----
    def conversion(x_old, y_old):
        x_new = -x_old
        y_new = y_old
        return x_new, y_new

    state_lock = threading.Lock()

    def load_model_and_vecnorm():
        nonlocal auto_model, auto_vecnorm, model_loaded
        # model_dir expected to be a folder path with model.zip and optional model_vector.pkl
        if model_dir is None:
            raise FileNotFoundError("Model directory not provided to multi_drone_move()")
        model_dir_str = str(Path(model_dir).expanduser())
        model_path = os.path.join(model_dir_str, "model.zip")
        vec_path = os.path.join(model_dir_str, "model_vector.pkl")

        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        print(f"[AUTO] Loading model from {model_path}")
        # load model with chosen device
        auto_model = SAC.load(model_path, device=device)
        auto_vecnorm = None
        if os.path.isfile(vec_path):
            try:
                try:
                    auto_vecnorm = VecNormalize.load(vec_path)
                except Exception:
                    with open(vec_path, "rb") as f:
                        auto_vecnorm = pickle.load(f)
            except Exception as e:
                print("[AUTO] Failed to load VecNormalize:", e)
                auto_vecnorm = None
        with state_lock:
            model_loaded = True
        print("[AUTO] Model loaded successfully.")

    def normalize_obs(obs):
        if obs is None:
            return obs
        try:
            with state_lock:
                vec = auto_vecnorm
            if vec is None:
                return obs
            if hasattr(vec, "normalize_obs"):
                try:
                    return vec.normalize_obs(np.array(obs, dtype=np.float32))
                except Exception:
                    pass
            if hasattr(vec, "obs_rms") and vec.obs_rms is not None:
                mean = getattr(vec.obs_rms, "mean", None)
                var = getattr(vec.obs_rms, "var", None)
                eps = getattr(vec.obs_rms, "epsilon", 1e-8)
                if mean is not None and var is not None and mean.shape == obs.shape:
                    return (obs - mean) / np.sqrt(var + eps)
            if hasattr(vec, "ob_rms") and vec.ob_rms is not None:
                mean = getattr(vec.ob_rms, "mean", None)
                var = getattr(vec.ob_rms, "var", None)
                eps = getattr(vec.ob_rms, "epsilon", 1e-8)
                if mean is not None and var is not None and mean.shape == obs.shape:
                    return (obs - mean) / np.sqrt(var + eps)
        except Exception as e:
            print("[AUTO] Obs normalization failed:", e)
        return obs

    def try_model_predict(obs_input):
        try:
            action, _ = auto_model.predict(obs_input, deterministic=True)
            action = np.array(action, dtype=np.float32).flatten()
            x_vel, y_vel = conversion(action[0], action[1])
            vel_convert = [x_vel, y_vel]
            action = np.array(vel_convert, dtype=np.float32).flatten()
            return action[:2]
        except Exception:
            try:
                arr = np.array(obs_input, dtype=np.float32)
                if arr.ndim == 1:
                    arr = arr[None, ...]
                action, _ = auto_model.predict(arr, deterministic=True)
                action = np.array(action, dtype=np.float32).flatten()
                x_vel, y_vel = conversion(action[0], action[1])
                vel_convert = [x_vel, y_vel]
                action = np.array(vel_convert, dtype=np.float32).flatten()
                return action[:2]
            except Exception as e:
                print("[AUTO] Predict error:", e)
                return np.array([0.0, 0.0], dtype=np.float32)

    def build_observation_for_leader():
        try:
            leader_node = drone_nodes[leader_id]
            vx = 0.0
            vy = 0.0
            if hasattr(leader_node.vehicle_local_position, "vx"):
                vx = float(leader_node.vehicle_local_position.vx)
            if hasattr(leader_node.vehicle_local_position, "vy"):
                vy = float(leader_node.vehicle_local_position.vy)
        except Exception:
            vx = 0.0
            vy = 0.0
        try:
            px = float(drone_nodes[leader_id].vehicle_local_position.x)
            py = float(drone_nodes[leader_id].vehicle_local_position.y)
        except Exception:
            px, py = 0.0, 0.0
        with state_lock:
            ld = drone_nodes[leader_id].lidar_data.copy() if isinstance(drone_nodes[leader_id].lidar_data, np.ndarray) else None
            gp = None if goal_pos is None else (goal_pos[0]-1, goal_pos[1])
            pa = prev_action.copy()
        if ld is None:
            lidar_vec = np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, dtype=np.float32)
        else:
            ld = np.array(ld, dtype=np.float32)
            if ld.shape[0] < LIDAR_NUM_RAYS:
                padded = np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, dtype=np.float32)
                padded[:ld.shape[0]] = ld
                lidar_vec = padded
            else:
                lidar_vec = ld[:LIDAR_NUM_RAYS]
        if gp is None:
            goal_dx = 0.0
            goal_dy = 0.0
        else:
            goal_dx = float(gp[0] - px)
            goal_dy = float(gp[1] - py)
        vx, vy = conversion(vx, vy)
        px, py = conversion(px, py)
        goal_dx, goal_dy = conversion(goal_dx, goal_dy)
        obs = np.concatenate((
            np.array([vx, vy, px, py], dtype=np.float32),
            lidar_vec.astype(np.float32),
            np.array([goal_dx, goal_dy], dtype=np.float32),
            pa.astype(np.float32)
        ))
        return obs

    def auto_loop():
        nonlocal prev_action, auto_running, leader_z_locked, goal_pos, at_goal, auto_model, auto_vecnorm, model_loaded
        kp = 0.5
        dt = 0.05
        step_ctr = 0
        while True:
            if not auto_running:
                time.sleep(0.01)
                continue
            try:
                with state_lock:
                    ml = model_loaded
                if not ml:
                    load_model_and_vecnorm()
            except Exception as e:
                print("[AUTO] Model loading error:", e)
                traceback.print_exc()
                auto_running = False
                time.sleep(1.0)
                continue

            leader_node = drone_nodes[leader_id]
            if leader_z_locked is None:
                try:
                    leader_z_locked = float(leader_node.vehicle_local_position.z)
                except Exception:
                    leader_z_locked = leader_node.current_setpoint[2] if len(leader_node.current_setpoint) > 2 else -5.0
                print(f"[AUTO] locking leader Z at {leader_z_locked}")

            if goal_pos is None:
                leader_node.flying_mode = 0
                hold_pos = [leader_node.vehicle_local_position.x, leader_node.vehicle_local_position.y, leader_z_locked]
                leader_node.current_setpoint = hold_pos
                leader_node.current_velocity = [0.0, 0.0, 0.0]
                time.sleep(dt)
                continue

            try:
                px = float(leader_node.vehicle_local_position.x)
                py = float(leader_node.vehicle_local_position.y)
                dist = float(np.hypot(goal_pos[0] - px, goal_pos[1] - py))
            except Exception:
                dist = float('inf')

            if dist < GOAL_THRESHOLD:
                if not at_goal:
                    print(f"[AUTO] goal reached at ({goal_pos[0]}, {goal_pos[1]}) dist={dist:.2f}. Holding position.")
                at_goal = True
                with state_lock:
                    prev_action[:] = 0.0
                leader_node.flying_mode = 0
                hold_pos = [px if px is not None else 0.0, py if py is not None else 0.0, leader_z_locked]
                leader_node.current_setpoint = hold_pos
                leader_node.current_velocity = [0.0, 0.0, 0.0]
                for drone_id, node in drone_nodes.items():
                    if drone_id == leader_id:
                        continue
                    desired_pos = [
                        hold_pos[0] + offsets[drone_id][0],
                        hold_pos[1] + offsets[drone_id][1],
                        hold_pos[2] + offsets[drone_id][2]
                    ]
                    pos_error = [0.0, 0.0, 0.0]
                    if is_valid_position(node.vehicle_local_position):
                        pos_error = [
                            desired_pos[0] - node.vehicle_local_position.x,
                            desired_pos[1] - node.vehicle_local_position.y,
                            desired_pos[2] - node.vehicle_local_position.z
                        ]
                    node.flying_mode = 0
                    node.current_velocity = [kp * e for e in pos_error]
                    node.current_setpoint = desired_pos
                time.sleep(dt)
                continue
            else:
                if at_goal:
                    print("[AUTO] Resuming model control (left goal or new goal).")
                at_goal = False

            obs = build_observation_for_leader()
            obs_norm = normalize_obs(obs)
            action_norm = try_model_predict(obs_norm)
            action_rawobs = try_model_predict(obs)
            action_used = action_norm.copy()
            action_used = np.array(action_used, dtype=np.float32).flatten()[:2]
            with state_lock:
                smoothed = action_smooth_alpha * prev_action + (1.0 - action_smooth_alpha) * action_used
                prev_action[:] = smoothed.copy()
            final_cmd = np.clip(prev_action.copy(), -MAX_VEL, MAX_VEL)
            vx_model = float(final_cmd[0])
            vy_model = float(final_cmd[1])
            leader_v_cmd = [vx_model, vy_model, 0.0]
            leader_node.flying_mode = 1
            leader_node.current_velocity = leader_v_cmd
            leader_node.current_setpoint = [float('nan'), float('nan'), float('nan')]

            leader_pos = [
                leader_node.vehicle_local_position.x,
                leader_node.vehicle_local_position.y,
                leader_z_locked
            ]
            for drone_id, node in drone_nodes.items():
                if drone_id == leader_id:
                    continue
                desired_pos = [
                    leader_pos[0] + offsets[drone_id][0],
                    leader_pos[1] + offsets[drone_id][1],
                    leader_pos[2] + offsets[drone_id][2]
                ]
                pos_error = [0.0, 0.0, 0.0]
                if is_valid_position(node.vehicle_local_position):
                    pos_error = [
                        desired_pos[0] - node.vehicle_local_position.x,
                        desired_pos[1] - node.vehicle_local_position.y,
                        desired_pos[2] - node.vehicle_local_position.z
                    ]
                node.flying_mode = 0
                node.current_velocity = [kp * e for e in pos_error]
                node.current_setpoint = desired_pos

            step_ctr += 1
            if DEBUG_PRINT and (step_ctr % max(1, DEBUG_PRINT_FREQ) == 0):
                with state_lock:
                    ld_copy = leader_node.lidar_data.copy() if isinstance(leader_node.lidar_data, np.ndarray) else np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE)
                    gp_local = None if goal_pos is None else (goal_pos[0], goal_pos[1])
                    yaw_local = leader_node.current_yaw
                min_idx = int(np.argmin(ld_copy)) if ld_copy.size > 0 else -1
                print(f"[AUTO DBG] px,py=({px:.2f},{py:.2f}) yaw_live={yaw_local:.3f} yaw_target={(leader_node.yaw_target if leader_node.yaw_target is not None else 'None')} goal=({gp_local[0] if gp_local else None},{gp_local[1] if gp_local else None}) dist={dist:.3f} lidar_min={ld_copy.min():.3f} min_idx={min_idx}")
                print(f"    action_from_norm={action_norm.tolist()}  action_from_rawobs={action_rawobs.tolist()}")
                print(f"    smoothed={smoothed.tolist()} final_cmd={final_cmd.tolist()} leader_cmd_world={leader_v_cmd} lidar_min={ld_copy.min():.2f} lidar_mean={ld_copy.mean():.2f}")
                try:
                    print(f"    obs_head={[float(x) for x in obs[:4]]} obs_norm_head={[float(x) for x in (obs_norm[:4] if obs_norm is not None else obs[:4])]} prev_action={prev_action.tolist()}")
                except Exception:
                    pass

            time.sleep(dt)

    # start the update and auto loop threads
    threading.Thread(target=update_loop, daemon=True).start()
    threading.Thread(target=auto_loop, daemon=True).start()

    # Start followers then leader, as before
    for drone_id_str, position in targets[0].items():
        drone_id = int(drone_id_str)
        world_x = position[0] - drone_id
        world_pos = [world_x, position[1], position[2]]
        run_node(drone_id, world_pos, is_leader=False)
        time.sleep(3.0)
    run_node(leader_id, center_of_mass, is_leader=True)

    print("Waiting for initial formation to stabilize...")
    stabilize_formation()

    print("\nFormation complete. Entering menu. Type 'manual' to start manual control or 'auto' to start auto mode.")
    print("While in AUTO mode: type 'move X Y' in this terminal to give goal.")
    print("Keyboard keys: W/A/S/D/I/J/K/L to control leader in manual mode; N to change formation; press 'z' (keyboard) to go back to menu from manual (listener) or type 'z' + Enter to stop auto from the terminal.")
    print(f"Drone nodes: {list(drone_nodes.keys())}")

    def command_input_loop():
        nonlocal manual_running, auto_running, goal_pos, leader_z_locked, at_goal, keyboard_thread_started, keyboard_thread
        while True:
            try:
                line = input().strip()
            except EOFError:
                time.sleep(0.1)
                continue
            if line == "":
                continue
            parts = line.split()
            cmd = parts[0].lower()
            if cmd == "manual":
                if auto_running:
                    auto_running = False
                    print("Stopping AUTO mode to start MANUAL.")
                manual_running = True
                leader_z_locked = None
                print("MANUAL started. Use WASD etc. Press 'z' key to return to menu (listener must be active).")
                if not keyboard_thread_started:
                    keyboard_thread = threading.Thread(target=keyboard_listener_thread, daemon=True)
                    keyboard_thread.start()
                    keyboard_thread_started = True
            elif cmd == "auto":
                if manual_running:
                    if keyboard_thread_started:
                        try:
                            stop_listening()
                        except Exception:
                            pass
                        keyboard_thread_started = False
                        keyboard_thread = None
                    manual_running = False
                    print("Stopping MANUAL to start AUTO.")
                auto_running = True
                goal_pos = None
                leader_z_locked = None
                at_goal = False
                print("AUTO started. Type 'move X Y' in this terminal to command the leader to go to that position.")
            elif cmd == "move":
                if len(parts) < 3:
                    print("Usage: move X Y")
                    continue
                try:
                    gx = float(parts[1])
                    gy = float(parts[2])
                except ValueError:
                    print("move: X and Y must be numbers.")
                    continue
                goal_pos = (gx, gy)
                at_goal = False
                print(f"[AUTO] new goal set to ({gx}, {gy})")
            elif cmd == "lockyaw":
                try:
                    drone_nodes[leader_id].yaw_target = float(drone_nodes[leader_id].current_yaw)
                    drone_nodes[leader_id].yaw_integral = 0.0
                    drone_nodes[leader_id].yaw_prev_error = 0.0
                    print(f"[CMD] yaw target locked at current yaw {drone_nodes[leader_id].yaw_target:.3f} rad ({drone_nodes[leader_id].yaw_target*180.0/math.pi:.1f} deg)")
                except Exception as e:
                    print("[CMD] Failed to lock yaw:", e)
            elif cmd == "unlockyaw":
                drone_nodes[leader_id].yaw_target = None
                drone_nodes[leader_id].yaw_integral = 0.0
                drone_nodes[leader_id].yaw_prev_error = 0.0
                print("[CMD] yaw control disabled (will no longer try to hold heading).")
            elif cmd == "setyaw":
                if len(parts) < 2:
                    print("Usage: setyaw <degrees>")
                    continue
                try:
                    deg = float(parts[1])
                    rad = deg * math.pi / 180.0
                    drone_nodes[leader_id].yaw_target = rad
                    drone_nodes[leader_id].yaw_integral = 0.0
                    drone_nodes[leader_id].yaw_prev_error = 0.0
                    print(f"[CMD] yaw target set to {rad:.3f} rad ({deg:.1f} deg)")
                except Exception as e:
                    print("[CMD] setyaw parse error:", e)
            elif cmd == "z":
                if manual_running:
                    manual_running = False
                    if keyboard_thread_started:
                        try:
                            stop_listening()
                        except Exception:
                            pass
                        keyboard_thread_started = False
                        keyboard_thread = None
                    for node in drone_nodes.values():
                        node.flying_mode = 0
                        node.current_velocity = [0.0, 0.0, 0.0]
                        node.current_yaw = 0.0
                    print("Exited MANUAL via typed 'z'. Returned to menu.")
                elif auto_running:
                    auto_running = False
                    print("Exiting AUTO mode via typed 'z'. Returned to menu.")
                else:
                    print("Not in MANUAL or AUTO mode.")
            elif cmd == "lidar":
                with state_lock:
                    raw = drone_nodes[leader_id].lidar_raw.copy() if isinstance(drone_nodes[leader_id].lidar_raw, np.ndarray) else None
                    shifted = drone_nodes[leader_id].lidar_shifted_raw.copy() if isinstance(drone_nodes[leader_id].lidar_shifted_raw, np.ndarray) else None
                    smooth = drone_nodes[leader_id].lidar_data.copy() if isinstance(drone_nodes[leader_id].lidar_data, np.ndarray) else None
                    a_min = drone_nodes[leader_id].lidar_angle_min
                    a_inc = drone_nodes[leader_id].lidar_angle_increment
                    computed_shift = drone_nodes[leader_id].lidar_computed_shift
                    yaw_target_local = drone_nodes[leader_id].yaw_target
                print("----- LIDAR DEBUG -----")
                print(f"LIDAR_INDEX_SHIFT (fallback const) = {LIDAR_INDEX_SHIFT}")
                print(f"lidar_computed_shift (used) = {computed_shift}")
                print(f"yaw_target = {yaw_target_local} rad ({(yaw_target_local*180.0/math.pi) if yaw_target_local is not None else None} deg)")
                if a_min is not None and a_inc is not None:
                    print(f"angle_min = {a_min:.4f} rad, angle_increment = {a_inc:.4f} rad ({a_inc * 180.0/np.pi:.3f} deg per ray)")
                else:
                    print("angle_min/angle_increment: not available")
                def pretty_print_array(name, arr):
                    if arr is None:
                        print(f"{name}: <no data>")
                        return
                    lines = []
                    for i, v in enumerate(arr):
                        if a_min is not None and a_inc is not None:
                            ang = a_min + i * a_inc
                            angd = ang * 180.0 / np.pi
                            lines.append(f"{i:02d}:{angd:6.1f}°:{v:.2f}")
                        else:
                            lines.append(f"{i:02d}:{v:.2f}")
                    print(f"{name} ({len(arr)}):")
                    chunk_size = 12
                    for i in range(0, len(lines), chunk_size):
                        print("  " + "  ".join(lines[i:i+chunk_size]))
                pretty_print_array("RAW (padded/truncated, before shift)", raw)
                pretty_print_array("SHIFTED (before smoothing)", shifted)
                pretty_print_array("SMOOTHED (used by model)", smooth)
                print("------------------------")
            elif cmd == "state":
                with state_lock:
                    yaw = drone_nodes[leader_id].current_yaw
                    px = getattr(drone_nodes[leader_id].vehicle_local_position, "x", None)
                    py = getattr(drone_nodes[leader_id].vehicle_local_position, "y", None)
                    vx = getattr(drone_nodes[leader_id].vehicle_local_position, "vx", None)
                    vy = getattr(drone_nodes[leader_id].vehicle_local_position, "vy", None)
                print("----- STATE -----")
                print(f"pos x,y = {px},{py}")
                print(f"vel vx,vy = {vx},{vy}")
                print(f"yaw_live (rad) = {yaw}  ({(yaw*180.0/np.pi) if yaw is not None else None:.2f} deg)")
                print(f"yaw_target (rad) = {drone_nodes[leader_id].yaw_target}  ({(drone_nodes[leader_id].yaw_target*180.0/np.pi) if drone_nodes[leader_id].yaw_target is not None else None:.2f} deg)")
                print("-----------------")
            elif cmd in ("exit", "quit"):
                print("Exiting program by user command.")
                break
            else:
                print(f"Unknown command: {line}")

    cmd_thread = threading.Thread(target=command_input_loop, daemon=True)
    cmd_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down all drones... (KeyboardInterrupt)")
    finally:
        for node in drone_nodes.values():
            try:
                node.land()
                time.sleep(1)
                node.disarm()
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser(description="Run multi-drone controller with optional model settings.")
    default_model_dir = Path(__file__).parent / "model"
    default_points = Path(__file__).parent / "points" / "drone_points.json"
    
    parser.add_argument(
        "-m", "--modelfile",
        type=str,
        default=str(default_model_dir),
        help="Path to directory containing model.zip and model_vector.pkl (default: ./model next to this script)."
    )
    parser.add_argument(
        "-d", "--device",
        type=str,
        choices=("cpu", "cuda"),
        default="cpu",
        help="Device to load model on ('cpu' or 'cuda'). Default: cpu."
    )
    parser.add_argument(
        "-p", "--points",
        type=str,
        default=str(default_points),
        help="Path to drone_points.json file or directory containing it. Default: ./points/drone_points.json next to this script."
    )

    args = parser.parse_args()

    model_dir_arg = str(Path(args.modelfile).expanduser())
    device_arg = args.device

    # Resolve points path (accepts either a file path or a directory)
    points_path = Path(args.points).expanduser()
    if points_path.is_dir():
        points_path = points_path / "drone_points.json"

    if not points_path.exists():
        print(f"File not found: {points_path}")
        return

    with points_path.open("r") as f:
        drone_targets = json.load(f)

    max_index = max(int(k) for k in drone_targets.keys())
    targets_list = [drone_targets[str(i)] for i in range(max_index + 1)]

    # pass model_dir and device into the controller
    multi_drone_move(targets_list, model_dir=model_dir_arg, device=device_arg)

if __name__ == '__main__':
    main()
