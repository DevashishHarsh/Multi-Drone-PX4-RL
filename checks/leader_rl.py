#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
import threading
import time
import os
import numpy as np
import pickle
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize
import traceback
import math

# ----------------- Config -----------------
LIDAR_TOPIC = "/lidar/scan"
LIDAR_NUM_RAYS = 60
LIDAR_MAX_RANGE = 10.0
# Legacy constant fallback. If LaserScan provides angle_min/angle_increment we will compute the correct shift.
# Keep this only as a fallback for unusual setups.
LIDAR_INDEX_SHIFT = 30  # original comment suggested 15 (90°) — keep as fallback

MAX_VEL = 2.0               # clamp model outputs to ±2 m/s for safety
GOAL_THRESHOLD = 1         # meters — when closer than this, stop model actions
MODEL_DEVICE = "cuda"         # change to "cuda" if available
DEBUG_PRINT = True           # set True to print obs/action occasionally (debug)
DEBUG_PRINT_FREQ = 10
# ------------------------------------------

class OffboardControl(Node):
    def __init__(self, drone_id=1):
        super().__init__(f'offboard_cli_control_{drone_id}')
        self.drone_id = drone_id

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        prefix = f'/px4_{drone_id}/fmu'

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, f'{prefix}/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, f'{prefix}/in/vehicle_command', qos_pub)
        # subscribe to vehicle local position & status
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, f'{prefix}/out/vehicle_local_position', self.local_position_cb, qos_sub)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, f'{prefix}/out/vehicle_status', self.status_cb, qos_sub)

        # subscribe to lidar
        self.lidar_sub = self.create_subscription(LaserScan, LIDAR_TOPIC, self._lidar_cb, qos_sub)

        # local state
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_started = False
        self.current_setpoint = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0            # live yaw from vehicle_local_position
        # removed yaw_locked; use yaw_target + PID
        self.yaw_target = None            # desired yaw in radians (None = not enforcing)
        # PID parameters (tune these as necessary)
        self.yaw_kp = 1.2
        self.yaw_ki = 0.01
        self.yaw_kd = 0.02
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.yaw_integral_limit = 1.0     # limit integral windup (radians*sec)
        # timer period (same as create_timer below) used as dt for PID
        self.timer_period = 0.1

        self.offboard_counter = 0
        self.flying_mode = 0

        # lidar data placeholders
        self.lidar_data = None                 # smoothed & used by model (shifted)
        self.lidar_raw = None                  # padded/truncated raw (no shift)
        self.lidar_shifted_raw = None          # raw after shift (before smoothing)
        self.lidar_angle_min = None
        self.lidar_angle_increment = None
        self.lidar_computed_shift = None       # computed shift (index) used for rotation

        # timer to regularly publish setpoints
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

    # ---------- callbacks for subscriptions ----------
    def local_position_cb(self, msg: VehicleLocalPosition):
        """
        Store latest vehicle local position message. Extract yaw if present.
        We still update current_yaw but publishing uses yaw_target (PID) if set.
        """
        try:
            self.vehicle_local_position = msg
            # VehicleLocalPosition may have `yaw` or `yaw_computed` depending on msg version
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
        """
        Store latest vehicle status message.
        """
        try:
            self.vehicle_status = msg
        except Exception as e:
            print("[STATUS CB] Error storing vehicle status:", e)

    # ---------- helper publishers ----------
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
        """
        Correctly populate px4_msgs.msg.TrajectorySetpoint fields.
        TrajectorySetpoint uses fixed-size arrays for position & velocity:
            float32[3] position
            float32[3] velocity
        Setting a field to NaN means "do not control that field".
        """
        try:
            msg = TrajectorySetpoint()
            # timestamp optional - PX4 ignores if not set; leave as default
            # prepare position (NED). If any coordinate is None -> NaN (means "ignore")
            px = float(x) if (x is not None and not (isinstance(x, float) and math.isnan(x))) else float('nan')
            py = float(y) if (y is not None and not (isinstance(y, float) and math.isnan(y))) else float('nan')
            pz = float(z) if (z is not None and not (isinstance(z, float) and math.isnan(z))) else float('nan')

            # velocity entries
            vx_f = float(vx) if (vx is not None and not (isinstance(vx, float) and math.isnan(vx))) else float('nan')
            vy_f = float(vy) if (vy is not None and not (isinstance(vy, float) and math.isnan(vy))) else float('nan')
            vz_f = float(vz) if (vz is not None and not (isinstance(vz, float) and math.isnan(vz))) else float('nan')

            # assign into fixed-size arrays
            try:
                msg.position = [px, py, pz]
                msg.velocity = [vx_f, vy_f, vz_f]
            except Exception:
                # fallback: assign element-wise (some generators expose array('f'))
                try:
                    msg.position[0] = px
                    msg.position[1] = py
                    msg.position[2] = pz
                    msg.velocity[0] = vx_f
                    msg.velocity[1] = vy_f
                    msg.velocity[2] = vz_f
                except Exception:
                    print("[TIMER] Warning: couldn't assign position/velocity arrays directly.")

            # yaw / yawspeed - set yaw to the provided value (NaN = don't control)
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
        # set mode to offboard
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def timer_cb(self):
        # periodically publish offboard/control mode and setpoint
        try:
            self.publish_offboard_control_mode()
            # compute yaw_to_send using PID if yaw_target set, else use current yaw
            yaw_to_send = None
            if self.yaw_target is None:
                # do not command yaw; send current yaw so the message has a sensible value
                yaw_to_send = float(self.current_yaw)
                # reset PID terms so we don't accumulate while not controlling
                self.yaw_integral = 0.0
                self.yaw_prev_error = 0.0
            else:
                # PID compute (shortest angular difference)
                # error = yaw_target - current_yaw, wrapped to [-pi, pi]
                def wrap_to_pi(a):
                    return (a + math.pi) % (2 * math.pi) - math.pi
                error = wrap_to_pi(self.yaw_target - self.current_yaw)
                # integrate with clamping
                self.yaw_integral += error * self.timer_period
                # clamp integral
                if self.yaw_integral > self.yaw_integral_limit:
                    self.yaw_integral = self.yaw_integral_limit
                elif self.yaw_integral < -self.yaw_integral_limit:
                    self.yaw_integral = -self.yaw_integral_limit
                derivative = (error - self.yaw_prev_error) / self.timer_period
                pid_out = (self.yaw_kp * error) + (self.yaw_ki * self.yaw_integral) + (self.yaw_kd * derivative)
                # clamp the pid output to a safe yaw-step (radians)
                max_step = 0.5  # radians per command step — tune down if too aggressive
                if pid_out > max_step:
                    pid_out = max_step
                elif pid_out < -max_step:
                    pid_out = -max_step
                yaw_to_send = float(self.current_yaw + pid_out)
                self.yaw_prev_error = error

            # publish current setpoint & velocity using corrected routine
            # use yaw_to_send computed above
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

    # LIDAR callback with low-pass filter (keeps your smoothing)
    _LP_ALPHA = 0.2
    def _lidar_cb(self, msg: LaserScan):
        try:
            # store angle info for debugging
            try:
                self.lidar_angle_min = float(msg.angle_min)
                self.lidar_angle_increment = float(msg.angle_increment)
            except Exception:
                self.lidar_angle_min = None
                self.lidar_angle_increment = None

            # read ranges -> convert inf to max range
            new_ranges = np.array(msg.ranges, dtype=np.float32)
            inf_mask = np.isinf(new_ranges)
            if np.any(inf_mask):
                new_ranges[inf_mask] = LIDAR_MAX_RANGE

            # pad/truncate to expected size
            if new_ranges.shape[0] < LIDAR_NUM_RAYS:
                padded = np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE, dtype=np.float32)
                padded[:new_ranges.shape[0]] = new_ranges
                raw_padded = padded
            elif new_ranges.shape[0] > LIDAR_NUM_RAYS:
                raw_padded = new_ranges[:LIDAR_NUM_RAYS]
            else:
                raw_padded = new_ranges.copy()

            # store the raw (padded/truncated) before rotation
            self.lidar_raw = raw_padded.copy()

            # ---------- compute dynamic shift (maps training angle -pi to index 0) ----------
            desired_shift = int(LIDAR_INDEX_SHIFT)
            # (kept your commented-out angle_min method disabled by default)
            self.lidar_computed_shift = desired_shift

            # rotate so that rotated[0] corresponds to angle = -pi (training convention)
            rotated = np.roll(raw_padded, -desired_shift)

            # store rotated (pre-smoothed)
            self.lidar_shifted_raw = rotated.copy()

            # low-pass smoothing applied to rotated data
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

# ---------------- Single-leader auto controller ----------------
def leader_auto_main():
    rclpy.init()
    leader_node = OffboardControl(drone_id=1)

    # spin the node in a separate thread
    def spin_node():
        try:
            exec_ = rclpy.executors.SingleThreadedExecutor()
            exec_.add_node(leader_node)
            exec_.spin()
        except Exception as e:
            print("Exception in ROS spin:", e)
    threading.Thread(target=spin_node, daemon=True).start()

    def conversion(x_old , y_old):
        x_new = -x_old
        y_new = y_old
        return x_new , y_new

    # small wait for publishers/subscriptions to settle
    time.sleep(1.5)

    # takeoff to 0,0,-5
    takeoff_pos = [0.0, 0.0, -5.0]
    leader_node.current_setpoint = takeoff_pos.copy()
    print(f"[INIT] Leader taking off to {takeoff_pos}")
    for _ in range(20):
        leader_node.publish_offboard_control_mode()
        leader_node.publish_trajectory_setpoint(*takeoff_pos, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.1)

    leader_node.engage_offboard()
    leader_node.arm()
    time.sleep(2.0)

    # ensure leader holds takeoff position briefly
    leader_node.current_setpoint = takeoff_pos.copy()
    for _ in range(30):
        leader_node.publish_trajectory_setpoint(*takeoff_pos, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.05)

    print("[INIT] Entering AUTO mode (leader controlled by SAC model). Type 'move X Y' to set a goal in terminal.")
    # ----------------- Auto vars -----------------
    auto_running = True
    auto_model = None
    auto_vecnorm = None
    model_loaded = False
    prev_action = np.array([0.0, 0.0], dtype=np.float32)
    action_smooth_alpha = 0.4  # same as training
    goal_pos = None
    leader_z_locked = None
    at_goal = False
    state_lock = threading.Lock()
    step_ctr = 0

    # model load helper (searches model/ or cwd/model, same as you)
    def load_model_and_vecnorm():
        nonlocal auto_model, auto_vecnorm, model_loaded
        base_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(base_dir, "model")
        if not os.path.isdir(model_dir):
            model_dir = os.path.join(os.getcwd(), "model")
        model_path = None
        vec_path = None
        if os.path.isdir(model_dir):
            for fname in os.listdir(model_dir):
                if fname.endswith(".zip") and model_path is None:
                    model_path = os.path.join(model_dir, fname)
                if (fname.endswith(".pkl") or fname.endswith(".pkl.gz")) and vec_path is None:
                    vec_path = os.path.join(model_dir, fname)
        if model_path is None:
            raise FileNotFoundError(f"Model (.zip) not found in {model_dir}")
        print(f"[AUTO] Loading model from {model_path}")
        auto_model = SAC.load(model_path, device=MODEL_DEVICE)
        auto_vecnorm = None
        if vec_path is not None:
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

    # normalize observation helper (keeps your logic)
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
        """
        Expect observation as in training:
        [vx, vy, px, py, lidar(60), goal_dx, goal_dy, prev_action_x, prev_action_y]
        """
        try:
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
            px = float(leader_node.vehicle_local_position.x)
            py = float(leader_node.vehicle_local_position.y)
        except Exception:
            # if no valid position yet, assume zeros
            px, py = 0.0, 0.0

        with state_lock:
            ld = leader_node.lidar_data.copy() if isinstance(leader_node.lidar_data, np.ndarray) else None
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

        
        vx,vy = conversion(vx, vy)
        px,py = conversion(px, py)
        goal_dx, goal_dy = conversion(goal_dx, goal_dy)

        obs = np.concatenate((
            np.array([vx, vy, px, py], dtype=np.float32),
            lidar_vec.astype(np.float32),
            np.array([goal_dx, goal_dy], dtype=np.float32),
            pa.astype(np.float32)
        ))
        return obs

    # AUTO loop thread (single leader)
    def auto_loop():
        nonlocal prev_action, auto_running, leader_z_locked, goal_pos, at_goal, auto_model, auto_vecnorm, model_loaded, step_ctr
        kp = 0.5
        dt = 0.05
        while True:
            if not auto_running:
                time.sleep(0.01)
                continue
            # ensure model loaded
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

            # lock z on first entry (and also lock yaw here so we command constant yaw after takeoff)
            if leader_z_locked is None:
                try:
                    leader_z_locked = float(leader_node.vehicle_local_position.z)
                except Exception:
                    leader_z_locked = takeoff_pos[2]
                print(f"[AUTO] locking leader Z at {leader_z_locked}")
                # do NOT auto-lock yaw on startup; user must call 'lockyaw' or 'setyaw'
                # removed previous yaw_locked assignment here

            # If no goal: hold leader in place (velocity 0)
            if goal_pos is None:
                leader_node.flying_mode = 0
                hold_pos = [leader_node.vehicle_local_position.x, leader_node.vehicle_local_position.y, leader_z_locked]
                leader_node.current_setpoint = hold_pos
                leader_node.current_velocity = [0.0, 0.0, 0.0]
                time.sleep(dt)
                continue

            # compute leader pos & dist to goal
            try:
                px = float(leader_node.vehicle_local_position.x)
                py = float(leader_node.vehicle_local_position.y)
                dist = float(np.hypot(goal_pos[0] - px, goal_pos[1] - py))
            except Exception:
                dist = float('inf')

            # If close to goal -> stop model actions and hold position
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
                time.sleep(dt)
                continue
            else:
                if at_goal:
                    print("[AUTO] Resuming model control (left goal or new goal).")
                at_goal = False

            # Build observation and normalize
            obs = build_observation_for_leader()
            obs_norm = normalize_obs(obs)

            # Predict
            action_norm = try_model_predict(obs_norm)
            action_rawobs = try_model_predict(obs)

            action_used = action_norm.copy()
            action_used = np.array(action_used, dtype=np.float32).flatten()[:2]

            with state_lock:
                smoothed = action_smooth_alpha * prev_action + (1.0 - action_smooth_alpha) * action_used
                prev_action[:] = smoothed.copy()

            final_cmd = np.clip(prev_action.copy(), -MAX_VEL, MAX_VEL)
            vx_model = float(final_cmd[0])   # model output (world-frame velocity as in training)
            vy_model = float(final_cmd[1])   # model output (world-frame)

            # IMPORTANT: training used world-frame velocities in PyBullet (no yaw rotation),
            # so we send the model outputs directly as world/local-frame velocities to PX4.
            # Do NOT rotate by yaw here unless you intentionally want to change frames.
            leader_v_cmd = [vx_model, vy_model, 0.0]

            leader_node.flying_mode = 1
            leader_node.current_velocity = leader_v_cmd
            leader_node.current_setpoint = [float('nan'), float('nan'), float('nan')]

            # debug prints occasionally
            step_ctr += 1
            if DEBUG_PRINT and (step_ctr % max(1, DEBUG_PRINT_FREQ) == 0):
                with state_lock:
                    ld_copy = leader_node.lidar_data.copy() if isinstance(leader_node.lidar_data, np.ndarray) else np.full(LIDAR_NUM_RAYS, LIDAR_MAX_RANGE)
                    gp_local = None if goal_pos is None else (goal_pos[0], goal_pos[1])
                    yaw_local = leader_node.current_yaw
                min_idx = int(np.argmin(ld_copy)) if ld_copy.size > 0 else -1
                print(f"[AUTO DBG] px,py=({px if 'px' in locals() else None:.2f},{py if 'py' in locals() else None:.2f}) yaw_live={yaw_local:.3f} yaw_target={(leader_node.yaw_target if leader_node.yaw_target is not None else 'None')} goal=({gp_local[0] if gp_local else None},{gp_local[1] if gp_local else None}) dist={dist:.3f} lidar_min={ld_copy.min():.3f} min_idx={min_idx}")
                print()
                print(f"    action_from_norm={action_norm.tolist()}  action_from_rawobs={action_rawobs.tolist()}")
                print()
                print(f"    smoothed={smoothed.tolist()} final_cmd={final_cmd.tolist()} leader_cmd_world={leader_v_cmd} lidar_min={ld_copy.min():.2f} lidar_mean={ld_copy.mean():.2f}")
                print()
                try:
                    print(f"    obs_head={[float(x) for x in obs[:4]]} obs_norm_head={[float(x) for x in (obs_norm[:4] if obs_norm is not None else obs[:4])]} prev_action={prev_action.tolist()}")
                except Exception:
                    pass

            time.sleep(dt)

    # start the auto loop thread
    threading.Thread(target=auto_loop, daemon=True).start()

    # command input loop (terminal)
    def command_input_loop():
        nonlocal goal_pos, auto_running, at_goal
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
            if cmd == "move":
                if len(parts) < 3:
                    print("Usage: move X Y")
                    continue
                try:
                    gx = float(parts[1])
                    gy = float(parts[2])
                except ValueError:
                    print("move: X and Y must be numbers.")
                    continue
                with state_lock:
                    goal_pos = (gx, gy)
                    at_goal = False
                print(f"[AUTO] new goal set to ({gx}, {gy})")

            elif cmd == "lockyaw":
                # set yaw target to current live yaw (hold heading)
                try:
                    leader_node.yaw_target = float(leader_node.current_yaw)
                    # reset PID internals
                    leader_node.yaw_integral = 0.0
                    leader_node.yaw_prev_error = 0.0
                    print(f"[CMD] yaw target locked at current yaw {leader_node.yaw_target:.3f} rad ({leader_node.yaw_target*180.0/math.pi:.1f} deg)")
                except Exception as e:
                    print("[CMD] Failed to lock yaw:", e)

            elif cmd == "unlockyaw":
                leader_node.yaw_target = None
                leader_node.yaw_integral = 0.0
                leader_node.yaw_prev_error = 0.0
                print("[CMD] yaw control disabled (will no longer try to hold heading).")

            elif cmd == "setyaw":
                # set yaw explicitly in degrees or radians (deg assumed)
                if len(parts) < 2:
                    print("Usage: setyaw <degrees>")
                    continue
                try:
                    deg = float(parts[1])
                    rad = deg * math.pi / 180.0
                    leader_node.yaw_target = rad
                    # reset PID internals so control starts clean
                    leader_node.yaw_integral = 0.0
                    leader_node.yaw_prev_error = 0.0
                    print(f"[CMD] yaw target set to {rad:.3f} rad ({deg:.1f} deg)")
                except Exception as e:
                    print("[CMD] setyaw parse error:", e)

            elif cmd == "z":
                # stop auto mode
                auto_running = False
                print("[CMD] AUTO stopped via typed 'z'.")
            elif cmd in ("start", "auto"):
                auto_running = True
                print("[CMD] AUTO started.")
            elif cmd == "lidar":
                # print lidar arrays for debugging: raw, shifted (pre-smoothed), and smoothed (used by model)
                with state_lock:
                    raw = leader_node.lidar_raw.copy() if isinstance(leader_node.lidar_raw, np.ndarray) else None
                    shifted = leader_node.lidar_shifted_raw.copy() if isinstance(leader_node.lidar_shifted_raw, np.ndarray) else None
                    smooth = leader_node.lidar_data.copy() if isinstance(leader_node.lidar_data, np.ndarray) else None
                    a_min = leader_node.lidar_angle_min
                    a_inc = leader_node.lidar_angle_increment
                    computed_shift = leader_node.lidar_computed_shift
                    yaw_target_local = leader_node.yaw_target

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
                    # print index:angle_deg:value (rounded)
                    lines = []
                    for i, v in enumerate(arr):
                        if a_min is not None and a_inc is not None:
                            ang = a_min + i * a_inc
                            angd = ang * 180.0 / np.pi
                            lines.append(f"{i:02d}:{angd:6.1f}°:{v:.2f}")
                        else:
                            lines.append(f"{i:02d}:{v:.2f}")
                    print(f"{name} ({len(arr)}):")
                    # join into a few lines for readability
                    chunk_size = 12
                    for i in range(0, len(lines), chunk_size):
                        print("  " + "  ".join(lines[i:i+chunk_size]))

                pretty_print_array("RAW (padded/truncated, before shift)", raw)
                pretty_print_array("SHIFTED (before smoothing)", shifted)
                pretty_print_array("SMOOTHED (used by model)", smooth)
                print("------------------------")

            elif cmd == "state":
                # quick state dump to help debug frame misalignments
                with state_lock:
                    yaw = leader_node.current_yaw
                    px = getattr(leader_node.vehicle_local_position, "x", None)
                    py = getattr(leader_node.vehicle_local_position, "y", None)
                    vx = getattr(leader_node.vehicle_local_position, "vx", None)
                    vy = getattr(leader_node.vehicle_local_position, "vy", None)
                print("----- STATE -----")
                print(f"pos x,y = {px},{py}")
                print(f"vel vx,vy = {vx},{vy}")
                print(f"yaw_live (rad) = {yaw}  ({(yaw*180.0/np.pi) if yaw is not None else None:.2f} deg)")
                print(f"yaw_target (rad) = {leader_node.yaw_target}  ({(leader_node.yaw_target*180.0/np.pi) if leader_node.yaw_target is not None else None:.2f} deg)")
                print("-----------------")

            elif cmd in ("exit", "quit"):
                print("[CMD] Exiting program.")
                break
            else:
                print(f"Unknown command: {line}")

    cmd_thread = threading.Thread(target=command_input_loop, daemon=True)
    cmd_thread.start()

    # keep alive until user quits or rclpy shutdown
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: shutting down.")
    finally:
        # graceful shutdown
        try:
            leader_node.land()
            time.sleep(1)
            leader_node.disarm()
        except Exception:
            pass
        try:
            leader_node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    leader_auto_main()
