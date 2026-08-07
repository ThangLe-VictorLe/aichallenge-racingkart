#!/usr/bin/env python3

import yaml
from typing import List, Tuple, Optional, NamedTuple
import dataclasses
from scipy import sparse
from scipy.sparse import dia_matrix
import numpy as np
import copy
import os
import shutil
from datetime import datetime

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, Bool, Float32MultiArray, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose2D, Point, Vector3
from std_msgs.msg import ColorRGBA

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

# autoware
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import GearCommand
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros.v2x_vehicle_tracker import (
    V2XVehicleTracker,
    predictions_to_obstacles,
)

# Multi_Purpose_MPC
from multi_purpose_mpc_ros.core.map import Map, Obstacle
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path

# Project
from multi_purpose_mpc_ros.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros.obstacle_manager import ObstacleManager
from multi_purpose_mpc_ros.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand, PathConstraints, BorderCells
from multi_purpose_mpc_ros.tools.reference_velocity_configulator import ReferenceVelocityConfigulator
from multi_purpose_mpc_ros.tools.steer_gain_configurator import SteerGainConfigurator


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)

def array_to_ackermann_control_command(stamp, u: np.ndarray, acc: float) -> AckermannControlCommand:
    msg = AckermannControlCommand()
    msg.stamp = stamp
    msg.lateral.stamp = stamp
    msg.lateral.steering_tire_angle = u[1]
    msg.lateral.steering_tire_rotation_rate = 2.0
    msg.longitudinal.stamp = stamp
    msg.longitudinal.speed = u[0]
    msg.longitudinal.acceleration = acc
    return msg

def yaw_from_quaternion(q: Quaternion):
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z
    sqw = q.w * q.w

    # Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw) # normalization added from urdfom_headers

    if sarg <= -0.99999:
        yaw = -2. * np.arctan2(q.y, q.x)
    elif sarg >= 0.99999:
        yaw = 2. * np.arctan2(q.y, q.x)
    else:
        yaw = np.arctan2(2. * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz)

    return yaw

def odom_to_pose_2d(odom: Odometry) -> Pose2D:
    pose = Pose2D()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = yaw_from_quaternion(odom.pose.pose.orientation)

    return pose

@dataclasses.dataclass
class MPCConfig:
    N: int
    Q: dia_matrix
    R: dia_matrix
    QN: dia_matrix
    v_max: float
    a_min: float
    a_max: float
    ay_max: float
    delta_max: float
    steer_rate_max: float
    control_rate: float
    steering_tire_angle_gain_var: float
    accel_low_pass_gain: float
    steer_low_pass_gain: float
    wp_id_offset: int
    use_max_kappa_pred: bool
    use_csv_velocity: bool
    use_dynamic_steer_gain: bool


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h
    BUG_ACC = 400.0

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 100.0

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str] = None, steer_gain_config_path: Optional[str] = None) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        self.declare_parameter("use_obstacle_avoidance", False)
        self.declare_parameter("use_stats", False)

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.USE_OBSTACLE_AVOIDANCE = self.get_parameter("use_obstacle_avoidance").get_parameter_value().bool_value
        self.use_stats = self.get_parameter("use_stats").get_parameter_value().bool_value

        self._config_path = config_path
        self._ref_vel_config_path: Optional[str] = ref_vel_config_path
        self._steer_gain_config_path: Optional[str] = steer_gain_config_path
        self._cfg = self._load_config()
        self._odom: Optional[Odometry] = None
        self._enable_control = True
        self._initialize()
        self._setup_parameters_callback()
        self._setup_pub_sub()

        if self.use_sim_time:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("use_sim_time is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_BUG_ACC:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_BUG_ACC is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_OBSTACLE_AVOIDANCE:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_OBSTACLE_AVOIDANCE is enabled!")
            self.get_logger().warn("------------------------------------")

    def _load_config(self) -> NamedTuple:

        # logging content
        with open(self._config_path, "r") as f:
            config_content = f.read()
            self.get_logger().info(
                "\n" +
                "----- config.yaml -----\n"+
                config_content + "\n" +
                "-----------------------")

        if self._ref_vel_config_path is not None:
            with open(self._ref_vel_config_path, "r") as f:
                ref_vel_config_content = f.read()
                self.get_logger().info(
                    "\n" +
                    "----- ref_vel.yaml -----\n"+
                    ref_vel_config_content + "\n" +
                    "-----------------------")

        if self._steer_gain_config_path is not None:
            with open(self._steer_gain_config_path, "r") as f:
                steer_gain_config_content = f.read()
                self.get_logger().info(
                    "\n" +
                    "----- steer_gain.yaml -----\n"+
                    steer_gain_config_content + "\n" +
                    "-----------------------")

        with open(self._config_path, "r") as f:
            cfg: NamedTuple = convert_to_namedtuple(yaml.safe_load(f)) # type: ignore

        # Check if the files exist
        mandatory_files = [cfg.map.yaml_path, cfg.waypoints.csv_path] # type: ignore
        for file_path in mandatory_files:
            file_exists(self.in_pkg_share(file_path))
        return cfg

    def _create_reference_path_from_autoware_trajectory(self, trajectory: Trajectory) -> Optional[ReferencePath]:
        wp_x = [0] * len(trajectory.points)
        wp_y = [0] * len(trajectory.points)
        for i, p in enumerate(trajectory.points):
            wp_x[i] = p.pose.position.x
            wp_y[i] = p.pose.position.y

        cfg_ref_path = self._cfg.reference_path # type: ignore
        reference_path = ReferencePath(
            self._map,
            wp_x,
            wp_y,
            cfg_ref_path.resolution,
            cfg_ref_path.smoothing_distance,
            cfg_ref_path.max_width,
            cfg_ref_path.circular)

        mpc_config = self._mpc_cfg
        speed_profile_constraints = {
            "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
            "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}

        if not reference_path.compute_speed_profile(speed_profile_constraints):
            return None

        return reference_path

    def _setup_parameters_callback(self) -> None:
        def declatre_parameters():
            cfg_mpc = self._cfg.mpc
            self.declare_parameter("v_max", cfg_mpc.v_max)
            self.declare_parameter("steering_tire_angle_gain_var", cfg_mpc.steering_tire_angle_gain_var)
            self.declare_parameter("Q0", cfg_mpc.Q[0])
            self.declare_parameter("Q1", cfg_mpc.Q[1])
            self.declare_parameter("Q2", cfg_mpc.Q[2])
            self.declare_parameter("R0", cfg_mpc.R[0])
            self.declare_parameter("R1", cfg_mpc.R[1])
            self.declare_parameter("QN0", cfg_mpc.QN[0])
            self.declare_parameter("QN1", cfg_mpc.QN[1])
            self.declare_parameter("QN2", cfg_mpc.QN[2])

            mpc_cfg = self._mpc_cfg
            self.declare_parameter("ay_max", mpc_cfg.ay_max)
            self.declare_parameter("accel_low_pass_gain", mpc_cfg.accel_low_pass_gain)
            self.declare_parameter("steer_low_pass_gain", mpc_cfg.steer_low_pass_gain)
            self.declare_parameter("wp_id_offset", mpc_cfg.wp_id_offset)

        def param_cb(parameters):
            cfg_mpc = self._cfg.mpc # type: ignore
            mpc_cfg = self._mpc_cfg

            def update_Q(index: int, value: float):
                cfg_mpc.Q[index] = value
                mpc_cfg.Q = sparse.diags(cfg_mpc.Q)
                self._mpc.update_Q(mpc_cfg.Q)
                self.get_logger().warn(f"Q[{index}] was updated to '{value}'")

            def update_R(index: int, value: float):
                cfg_mpc.R[index] = value
                mpc_cfg.R = sparse.diags(cfg_mpc.R)
                self._mpc.update_R(mpc_cfg.R)
                self.get_logger().warn(f"R[{index}] was updated to '{value}'")

            def update_QN(index: int, value: float):
                cfg_mpc.QN[index] = value
                mpc_cfg.QN = sparse.diags(cfg_mpc.QN)
                self._mpc.update_QN(mpc_cfg.QN)
                self.get_logger().warn(f"QN[{index}] was updated to '{value}'")

            for param in parameters:
                if param.name == "v_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.v_max = param.value
                    self._mpc.update_v_max(kmh_to_m_per_sec(param.value))
                    v_ref: List[float] = [kmh_to_m_per_sec(param.value)] * len(self._reference_path.waypoints)
                    self._reference_path.set_v_ref(v_ref)

                    self.get_logger().warn(f"v_max was updated to '{param.value}' [km/h]")

                elif param.name == "steering_tire_angle_gain_var" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steering_tire_angle_gain_var = param.value
                    self.get_logger().warn(f"steering_tire_angle_gain_var was updated to '{param.value}'")

                elif param.name == "Q0" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(0, param.value)
                elif param.name == "Q1" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(1, param.value)
                elif param.name == "Q2" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(2, param.value)


                elif param.name == "R0" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(0, param.value)
                elif param.name == "R1" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(1, param.value)

                elif param.name == "QN0" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(0, param.value)
                elif param.name == "QN1" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(1, param.value)
                elif param.name == "QN2" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(2, param.value)

                elif param.name == "ay_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.ay_max = param.value
                    self._mpc.update_ay_max(param.value)
                    self.get_logger().warn(f"ay_max was updated to '{param.value}'")

                elif param.name == "accel_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.accel_low_pass_gain = param.value
                    self.get_logger().warn(f"accel_low_pass_gain was updated to '{param.value}'")

                elif param.name == "steer_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steer_low_pass_gain = param.value
                    self.get_logger().warn(f"steer_low_pass_gain was updated to '{param.value}'")

                elif param.name == "wp_id_offset" and param.type_ == Parameter.Type.INTEGER:
                    mpc_cfg.wp_id_offset = param.value
                    self._mpc.update_wp_id_offset(param.value)
                    self.get_logger().warn(f"wp_id_offset was updated to '{param.value}'")


            return SetParametersResult(successful=True)

        declatre_parameters()
        self.add_on_set_parameters_callback(param_cb)

    def _initialize(self) -> None:
        def create_map() -> Map:
            return Map(self.in_pkg_share(self._cfg.map.yaml_path)) # type: ignore

        def create_ref_path(map: Map) -> ReferencePath:
            cfg_ref_path = self._cfg.reference_path # type: ignore

            is_ref_path_given = cfg_ref_path.csv_path != "" # type: ignore
            if is_ref_path_given:
                print("Using given reference path")
                wp_x, wp_y, _, _, wp_v = load_ref_path(self.in_pkg_share(self._cfg.reference_path.csv_path)) # type: ignore
                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    wp_v,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)

            else:
                print("Using waypoints to create reference path")
                wp_x, wp_y = load_waypoints(self.in_pkg_share(self._cfg.waypoints.csv_path)) # type: ignore
                wp_v = [30.0] * len(wp_x)

                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    wp_v,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)


        def create_obstacles() -> List[Obstacle]:
            use_csv_obstacles = self._cfg.obstacles.csv_path != "" # type: ignore
            if use_csv_obstacles:
                obstacles_file_path = self.in_pkg_share(self._cfg.obstacles.csv_path) # type: ignore
                obs_x, obs_y = load_waypoints(obstacles_file_path)
                obstacles = []
                for cx, cy in zip(obs_x, obs_y):
                    obstacles.append(Obstacle(cx=cx, cy=cy, radius=self._cfg.obstacles.radius)) # type: ignore
                self._obstacle_manager = ObstacleManager(self._map, obstacles)
                return obstacles
            else:
                return []

        def create_car(ref_path: ReferencePath) -> BicycleModel:
            cfg_model = self._cfg.bicycle_model # type: ignore
            return BicycleModel(
                ref_path,
                cfg_model.length,
                cfg_model.width,
                1.0 / self._cfg.mpc.control_rate) # type: ignore

        def create_mpc(car: BicycleModel) -> Tuple[MPCConfig, MPC]:
            cfg_mpc = self._cfg.mpc # type: ignore

            mpc_cfg = MPCConfig(
                cfg_mpc.N,
                sparse.diags(cfg_mpc.Q),
                sparse.diags(cfg_mpc.R),
                sparse.diags(cfg_mpc.QN),
                kmh_to_m_per_sec(self.BUG_VEL if self.USE_BUG_ACC else cfg_mpc.v_max),
                cfg_mpc.a_min,
                cfg_mpc.a_max,
                cfg_mpc.ay_max,
                np.deg2rad(cfg_mpc.delta_max_deg),
                cfg_mpc.steer_rate_max,
                cfg_mpc.control_rate,
                cfg_mpc.steering_tire_angle_gain_var,
                cfg_mpc.accel_low_pass_gain,
                cfg_mpc.steer_low_pass_gain,
                cfg_mpc.wp_id_offset,
                cfg_mpc.use_max_kappa_pred,
                cfg_mpc.use_csv_velocity,
                cfg_mpc.use_dynamic_steer_gain)

            state_constraints = {
                "xmin": np.array([-np.inf, -np.inf, -np.inf]),
                "xmax": np.array([np.inf, np.inf, np.inf])}
            input_constraints = {
                "umin": np.array([0.0, -np.tan(mpc_cfg.delta_max) / car.length]),
                "umax": np.array([mpc_cfg.v_max, np.tan(mpc_cfg.delta_max) / car.length])}

            # mpcからのsteer指令出力は、gainを掛けて出力され、その状態で車体のsteer rate limit が適用されるため、
            # mpcの制御計算におけるsteer_rate_maxは、実際のsteer_rate_maxをgainで除した値で設定する
            scaled_steer_rate_max = mpc_cfg.steer_rate_max / mpc_cfg.steering_tire_angle_gain_var

            mpc = MPC(
                car,
                mpc_cfg.N,
                mpc_cfg.Q,
                mpc_cfg.R,
                mpc_cfg.QN,
                state_constraints,
                input_constraints,
                mpc_cfg.ay_max,
                scaled_steer_rate_max,
                mpc_cfg.wp_id_offset,
                self.USE_OBSTACLE_AVOIDANCE,
                self._cfg.reference_path.use_path_constraints_topic,
                mpc_cfg.use_max_kappa_pred)

            return mpc_cfg, mpc

        def compute_speed_profile(car: BicycleModel, mpc_config: MPCConfig) -> None:
            speed_profile_constraints = {
                "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
                "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}
            car.reference_path.compute_speed_profile(speed_profile_constraints)

        def create_ref_vel_configulator() -> Optional[ReferenceVelocityConfigulator]:
            if self._ref_vel_config_path is None:
                return None
            return ReferenceVelocityConfigulator(self, self._config_path, self._ref_vel_config_path)

        def create_steer_gain_configurator() -> Optional[SteerGainConfigurator]:
            if self._steer_gain_config_path is None:
                return None
            return SteerGainConfigurator(self, self._config_path, self._steer_gain_config_path)

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg, self._mpc = create_mpc(self._car)
        compute_speed_profile(self._car, self._mpc_cfg)

        if self.USE_OBSTACLE_AVOIDANCE:
            # Pre-build the static path constraints table so it is always available
            # as a clean fallback when obstacle avoidance is temporarily suppressed
            # (recovery, cooldown, stopped). Without this the static-bounds branch in
            # MPC._init_problem reads from an empty array and crashes.
            self._reference_path.update_simple_path_constraints(
                int(self._mpc_cfg.N),
                self._car.safety_margin)

        if self._mpc_cfg.use_csv_velocity:
            self.get_logger().info("Using CSV vx_mps for velocity (ReferenceVelocityConfigulator disabled)")
            self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = None
        else:
            self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()

        if self._mpc_cfg.use_dynamic_steer_gain:
            self._steer_gain_configurator: Optional[SteerGainConfigurator] = create_steer_gain_configurator()
        else:
            self.get_logger().info("Using fixed steer gain from config (SteerGainConfigurator disabled)")
            self._steer_gain_configurator: Optional[SteerGainConfigurator] = None

        self._trajectory: Optional[Trajectory] = None
        self._path_constraints = None

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = bool(self._static_obstacles)
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)  # type: ignore
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(mpc_N)
            ]
            # コリドー外の V2X 障害物で MPC のコリドー狭窄/反転が起きないよう、
            # ref-path 近傍のみに絞り込む。閾値 = max_width/2 + vehicle_radius + 余白。
            ref_max_width = float(self._cfg.reference_path.max_width)  # type: ignore
            self._v2x_corridor_threshold_sq = (
                ref_max_width / 2.0 + self._v2x_vehicle_radius + 0.5
            ) ** 2
            wps = self._reference_path.waypoints
            self._waypoint_xy = np.asarray(
                [(wp.x, wp.y) for wp in wps], dtype=np.float64)
            # Build cumulative arc-length array for Frenet-frame proximity check
            diffs = np.diff(self._waypoint_xy, axis=0)
            seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
            self._waypoint_s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
            self._total_track_s = float(self._waypoint_s[-1])
            # Proximity gate params (with hysteresis)
            self._v2x_activation_dist = float(v2x_cfg.activation_distance_m)  # type: ignore
            self._v2x_deactivation_dist = float(v2x_cfg.deactivation_distance_m)  # type: ignore
            self._overtaking_steer_gain = float(v2x_cfg.overtaking_steer_gain)  # type: ignore
            # Runtime state
            self._v2x_avoidance_active = False

        # --- Debug Visualization ---
        # Read debug_visualization section from config (all flags default to False).
        # Each flag can be toggled without rebuild — just change config.yaml and restart.
        _dbg_cfg = getattr(self._cfg, 'debug_visualization', None)
        self._dbg_v2x_log        = bool(getattr(_dbg_cfg, 'v2x_diagnostic_log', False))
        self._dbg_corridor        = bool(getattr(_dbg_cfg, 'path_corridor', False))
        self._dbg_dyn_obstacles   = bool(getattr(_dbg_cfg, 'dynamic_obstacles', False))
        self._dbg_v2x_status_text = bool(getattr(_dbg_cfg, 'v2x_status_text', False))
        self._dbg_pred_colors     = bool(getattr(_dbg_cfg, 'prediction_colors', False))
        self._v2x_dbg_counter     = 0

        # Laps
        self._current_laps = 1
        self._last_lap_time = 0.0
        self._lap_times = [None] * (self.MAX_LAPS + 1) # +1 means include lap 0

        # condition
        self._last_condition = None
        self._last_colliding_time = None

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

        # Recovery state machine initialization
        recovery_cfg = getattr(self._cfg, 'recovery', None)
        if recovery_cfg is not None:
            self._recovery_enabled = bool(getattr(recovery_cfg, 'enable', True))
            # stuck threshold: odom velocity is in m/s
            self._recovery_stuck_vel_threshold = float(getattr(recovery_cfg, 'stuck_vel_threshold_kmh', 0.5)) / 3.6
            self._recovery_stuck_time_sec = float(getattr(recovery_cfg, 'stuck_time_sec', 1.0))
            # u[0] (speed target) is in km/h — no unit conversion needed
            self._recovery_reverse_vel = float(getattr(recovery_cfg, 'reverse_vel_kmh', -5.0))
            self._recovery_reverse_time_sec = float(getattr(recovery_cfg, 'reverse_time_sec', 1.5))
            self._recovery_steer_gain = float(getattr(recovery_cfg, 'recovery_steer_gain', 1.0))
            self._post_recovery_steer_gain = float(getattr(recovery_cfg, 'post_recovery_steer_gain', 1.0))
            self._recovery_cooldown_sec = float(getattr(recovery_cfg, 'cooldown_sec', 2.0))
            raw_disabled = getattr(recovery_cfg, 'disabled_waypoints', None)
            if raw_disabled is None:
                raw_disabled = getattr(recovery_cfg, 'disabled_waypoint_ranges', None)
        else:
            self._recovery_enabled = True
            self._recovery_stuck_vel_threshold = 0.5 / 3.6  # m/s
            self._recovery_stuck_time_sec = 1.0
            self._recovery_reverse_vel = -5.0  # km/h (same unit as u[0])
            self._recovery_reverse_time_sec = 1.5
            self._recovery_steer_gain = 1.0
            self._post_recovery_steer_gain = 1.0
            self._recovery_cooldown_sec = 2.0
            raw_disabled = None

        self._recovery_disabled_ranges: List[Tuple[int, int]] = []
        if raw_disabled is not None:
            if isinstance(raw_disabled, list):
                if len(raw_disabled) == 2 and all(isinstance(x, (int, float)) for x in raw_disabled):
                    self._recovery_disabled_ranges.append((int(raw_disabled[0]), int(raw_disabled[1])))
                else:
                    for item in raw_disabled:
                        if isinstance(item, (list, tuple)) and len(item) == 2:
                            self._recovery_disabled_ranges.append((int(item[0]), int(item[1])))
                        elif isinstance(item, (int, float)):
                            self._recovery_disabled_ranges.append((int(item), int(item)))

        self._recovery_state = "NORMAL"
        self._stuck_start_time = None
        self._recovering_start_time = None
        self._cooldown_start_time = None
        # Only allow stuck detection after the kart has moved at least once
        # (prevents false trigger at race start when v=0 but MPC commands speed)
        self._has_moved = False

        # save config
        if self._cfg.common.save_config:
            self._save_config()

    def _save_config(self) -> None:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst_dir = self.PKG_PATH + f"log/{now}"
        os.makedirs(dst_dir, exist_ok=True)
        shutil.copy(self._config_path, os.path.join(dst_dir, "config.yaml"))

    def _setup_pub_sub(self) -> None:
        # Publishers
        if self.USE_BUG_ACC:
          self._command_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command", 1)
        else:
          self._command_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd_raw", 1)
          print("use normal ackermann control command")

        # Gear command publisher for recovery (DRIVE/REVERSE)
        self._gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", 1)

        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._mpc_pred_pub = self.create_publisher(
            MarkerArray, "/mpc/prediction", 1)
        self._mpc_pred_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall", 1)

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._ref_path_pub = self.create_publisher(
            MarkerArray, "/mpc/ref_path", latching_qos)
        self._ref_path_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound", latching_qos)

        # Debug visualization publishers (only created when USE_OBSTACLE_AVOIDANCE is True
        # because corridor/dynamic-obstacle features are only meaningful with V2X active).
        if self.USE_OBSTACLE_AVOIDANCE:
            self._v2x_status_pub = self.create_publisher(
                MarkerArray, "/mpc/v2x_status", 1)
            self._corridor_pub = self.create_publisher(
                MarkerArray, "/mpc/path_corridor", 1)
            self._dyn_obs_pub = self.create_publisher(
                MarkerArray, "/mpc/dynamic_obstacles", 1)

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)
        self._control_mode_request_sub = self.create_subscription(
            Bool, "control/control_mode_request_topic", self._control_mode_request_callback, 1)
        # simple_trajectory_generator publishes with BEST_EFFORT/KEEP_LAST(1) — match it
        # so the subscription is QoS-compatible (rclpy default is RELIABLE).
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._trajectory_sub = self.create_subscription(
            Trajectory, "planning/scenario_planning/trajectory", self._trajectory_callback, trajectory_qos)
        self._stop_request_sub = self.create_subscription(
            Empty, "/control/mpc/stop_request", self._stop_request_callback, 1)

        if self.use_sim_time:
            self._awsim_status_sub = self.create_subscription(
                Float32MultiArray, "/awsim/status", self._awsim_status_callback, 1)
            self._condition_sub = self.create_subscription(
                Int32, "/aichallenge/pitstop/condition", self._condition_callback, 1)

        if self.USE_OBSTACLE_AVOIDANCE:
            if self._cfg.reference_path.use_path_constraints_topic: # type: ignore
                self._path_constraints_sub = self.create_subscription(
                    PathConstraints, "/path_constraints_provider/path_constraints", self._path_constraints_callback, 1)

            if self._cfg.reference_path.use_border_cells_topic: # type: ignore
                self._border_cells_sub = self.create_subscription(
                    BorderCells, "/path_constraints_provider/border_cells", self._border_cells_callback, 1)

            self._v2x_sub = self.create_subscription(
                V2XVehiclePositionArray,
                "/v2x/vehicle_positions",
                self._v2x_callback,
                1)

    def _create_ackerman_control_command(self, stamp, u, acc, bug_acc_enabled):
        v_cmd = u[0]
        steer_cmd = u[1]

        ackerman_cmd = array_to_ackermann_control_command(stamp.to_msg(), [v_cmd, steer_cmd], acc)

        if not self.USE_BUG_ACC:
            return ackerman_cmd

        ackerman_boost_cmd = AckermannControlBoostCommand()
        ackerman_boost_cmd.command = ackerman_cmd
        ackerman_boost_cmd.boost_mode = bug_acc_enabled
        return ackerman_boost_cmd

    def _publish_control_command(self, stamp, u, acc, bug_acc_enabled):
        cmd = self._create_ackerman_control_command(stamp, u, acc, bug_acc_enabled)

        # publish raw control command
        self._command_raw_pub.publish(cmd)

        # compensate steering angle for the real vehicle
        # AWSIMにおいても後段のactuation_cmd_converter でgainを考慮した指令を生成するため、実機/sim問わず
        # gain を掛ける
        cmd.lateral.steering_tire_angle *= self._mpc_cfg.steering_tire_angle_gain_var
        self._command_pub.publish(cmd)


    def _is_recovery_disabled_at_wp(self, wp_id: int) -> bool:
        for start_wp, end_wp in self._recovery_disabled_ranges:
            if start_wp <= wp_id <= end_wp:
                return True
        return False

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _control_mode_request_callback(self, msg):
        if msg.data and not self._enable_control:
            self.get_logger().info("Control mode request received")
            self._enable_control = True
            self._has_moved = False
            self._recovery_state = "NORMAL"
            self._stuck_start_time = None
        elif not msg.data and self._enable_control:
            self._enable_control = False
            self._has_moved = False
            self._recovery_state = "NORMAL"
            self._stuck_start_time = None

    def _path_constraints_callback(self, msg: PathConstraints):
        self._reference_path.set_path_constraints(
            msg.upper_bounds, msg.lower_bounds, msg.rows, msg.cols)

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x_tracker.update(msg)

        # === PROXIMITY GATE WITH HYSTERESIS ===
        # Only activate the heavy update_path_constraints path when an opponent
        # is detected AHEAD of us within activation_distance_m along the raceline.
        # Deactivate only when all opponents are beyond deactivation_distance_m
        # (hysteresis prevents rapid toggling at the boundary).
        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        ego_s = self._waypoint_s[min(ego_wp_id, len(self._waypoint_s) - 1)]
        track_len = self._total_track_s
        activate_threshold = self._v2x_activation_dist
        deactivate_threshold = self._v2x_deactivation_dist

        any_opponent_in_activate_range = False
        any_opponent_in_deactivate_range = False
        for v in msg.vehicles:
            opp_x = float(v.position.x)
            opp_y = float(v.position.y)
            # Snap opponent to nearest waypoint
            dxy = self._waypoint_xy - np.array([opp_x, opp_y], dtype=np.float64)
            dists_sq = np.einsum('ij,ij->i', dxy, dxy)
            nearest_wp_id = int(np.argmin(dists_sq))
            # Skip if this is the ego vehicle (same position as us)
            if np.sqrt(dists_sq[nearest_wp_id]) < 0.1:
                continue
            opp_s = self._waypoint_s[nearest_wp_id]
            # Signed forward distance along raceline; wraps cleanly on circular track
            ds = (opp_s - ego_s) % track_len
            # ds in (0, track_len): small = just ahead, large = behind us
            if 0.0 < ds <= activate_threshold:
                any_opponent_in_activate_range = True
                break
            if 0.0 < ds <= deactivate_threshold:
                any_opponent_in_deactivate_range = True

        # Apply hysteresis: activate on close approach, deactivate only when all clear
        prev_active = self._v2x_avoidance_active
        if any_opponent_in_activate_range:
            self._v2x_avoidance_active = True
        elif not any_opponent_in_deactivate_range:
            self._v2x_avoidance_active = False
        # else: opponent is between activate and deactivate thresholds — hold current state

        if self._v2x_avoidance_active != prev_active:
            if self._v2x_avoidance_active:
                self.get_logger().warn(
                    "[V2X] Overtaking mode ACTIVATED — opponent within "
                    f"{activate_threshold:.0f} m ahead. Steer gain -> {self._overtaking_steer_gain}")
            else:
                self.get_logger().info(
                    "[V2X] Overtaking mode DEACTIVATED — no opponent within "
                    f"{deactivate_threshold:.0f} m ahead. Resuming normal steer gain.")

        # Store last message for _publish_v2x_status (needs vehicle IDs)
        self._last_v2x_msg = msg

        if self._v2x_avoidance_active:
            predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
            self._dynamic_obstacles = predictions_to_obstacles(
                predictions, self._v2x_vehicle_radius)
            # Only trigger map update if not in recovery — wall proximity makes solver infeasible
            if self._recovery_state not in ("RECOVERING", "COOLDOWN"):
                self._obstacles_updated = True
        else:
            # Opponent left range — clear dynamic obstacles so map resets to clean static corridor
            if self._dynamic_obstacles:
                self._dynamic_obstacles = []
                self._obstacles_updated = True  # trigger map reset to clear old obstacle blobs

        # --- V2X diagnostic terminal log (throttled, enabled via config) ---
        if self._dbg_v2x_log:
            self._v2x_diagnostic_log(
                msg, ego_wp_id, ego_s, track_len,
                activate_threshold, deactivate_threshold)

    def _filter_obstacles_to_corridor(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        if not obstacles or self._waypoint_xy.size == 0:
            return obstacles
        thr_sq = self._v2x_corridor_threshold_sq
        wps = self._waypoint_xy
        kept: List[Obstacle] = []
        for ob in obstacles:
            dxy = wps - np.array([ob.cx, ob.cy], dtype=np.float64)
            if np.min(np.einsum('ij,ij->i', dxy, dxy)) <= thr_sq:
                kept.append(ob)
        return kept

    def _border_cells_callback(self, msg: BorderCells):
        self._reference_path.set_border_cells(
            msg.dynamic_upper_bounds, msg.dynamic_lower_bounds, msg.rows, msg.cols)

    def _trajectory_callback(self, msg):
        self._trajectory = msg

    def _awsim_status_callback(self, msg):
        laps = int(msg.data[1])
        lap_time = msg.data[2]
        # section = int(msg.data[3])

        if self._current_laps is None:
            self._current_laps = 1 if laps == 0 else laps

        if laps > self._current_laps:
            self.get_logger().info(f'\033[32mLap {self._current_laps} completed! Lap time: {self._last_lap_time} s\033[0m')
            self._lap_times[self._current_laps] = self._last_lap_time
            self._current_laps = laps
            # Notify SteerGainConfigurator so it switches to warm-tire gains from lap 2 onward
            if self._steer_gain_configurator is not None:
                self._steer_gain_configurator.set_lap(self._current_laps)

        self._last_lap_time = lap_time

    def _condition_callback(self, msg: Int32):
        if self._last_condition is None:
            self._last_condition = msg.data

        diff_condition = msg.data - self._last_condition
        if diff_condition > 30.0:
            self._last_colliding_time = self.get_clock().now()
            self.get_logger().warning(f"Collision detected!")
        self._last_condition = msg.data

    def _stop_request_callback(self, msg: Empty) -> None:
        if self._enable_control:
            self.get_logger().warn(f"Stop request received {self._enable_control}")
            self._enable_control = False

    def _wait_until_clock_received(self) -> None:
        if self.use_sim_time:
            self.get_logger().info(f"wait until clock received...")
            rate = self.create_rate(10)
            rate.sleep()
            self.get_logger().info(f">> OK!")

    def _wait_until_message_received(self, message_getter, message_name: str, timeout: float, rate_hz: int = 30) -> None:

        t_start = self.get_clock().now()
        rate = self.create_rate(rate_hz)

        self.get_logger().info(f"wait until {message_name} received...")

        while message_getter() is None:
            now = self.get_clock().now()
            if (now - t_start).nanoseconds > timeout * 1e9:
                self.get_logger().info(f"now: {now}, t_start: {t_start}")
                raise TimeoutError(f"Timeout while waiting for {message_name} message")
            rate.sleep()

        self.get_logger().info(f">> OK!")

    def _wait_until_odom_received(self, timeout: float = 30.) -> None:
        self._wait_until_message_received(lambda: self._odom, 'odometry', timeout)

    def _wait_until_trajectory_received(self, timeout: float = 30.) -> None:
        if self._cfg.reference_path.update_by_topic:
            self._wait_until_message_received(lambda: self._trajectory, 'trajectory', timeout)

    def _wait_until_path_constraints_received(self, timeout: float = 30.) -> None:
        if self.USE_OBSTACLE_AVOIDANCE and self._cfg.reference_path.use_path_constraints_topic: # type: ignore
            self._wait_until_message_received(lambda: self._reference_path.path_constraints, 'path constraints', timeout)

    # =========================================================================
    # Debug Visualization Helpers
    # =========================================================================

    def _v2x_diagnostic_log(self, msg, ego_wp_id, ego_s, track_len,
                             activate_threshold, deactivate_threshold) -> None:
        """Throttled terminal log: prints V2X vehicle positions/distances and gate state."""
        self._v2x_dbg_counter += 1
        if self._v2x_dbg_counter % 10 != 1:
            return
        lines = [
            f"[V2X DBG] vehicles={len(msg.vehicles)}  "
            f"ego_wp={ego_wp_id}  ego_s={ego_s:.1f}m"
        ]
        for v in msg.vehicles:
            opp_x = float(v.position.x)
            opp_y = float(v.position.y)
            dxy = self._waypoint_xy - np.array([opp_x, opp_y], dtype=np.float64)
            dists_sq = np.einsum('ij,ij->i', dxy, dxy)
            nearest_wp = int(np.argmin(dists_sq))
            nearest_dist = float(np.sqrt(dists_sq[nearest_wp]))
            opp_s = self._waypoint_s[nearest_wp]
            ds = (opp_s - ego_s) % track_len
            is_ego = nearest_dist < 0.8
            lines.append(
                f"  vid={v.vehicle_id}  pos=({opp_x:.1f},{opp_y:.1f})"
                f"  wp={nearest_wp}  dist_to_wp={nearest_dist:.2f}m"
                f"  ds={ds:.1f}m ({'BEHIND' if ds > track_len / 2 else 'AHEAD'})"
                + ("  [EGO SKIP]" if is_ego else "")
            )
        lines.append(
            f"  gate={'ACTIVE' if self._v2x_avoidance_active else 'INACTIVE'}"
            f"  activate<{activate_threshold:.0f}m"
            f"  deactivate<{deactivate_threshold:.0f}m"
            f"  dyn_obs={len(self._dynamic_obstacles)}"
        )
        self.get_logger().info('\n'.join(lines))

    def _publish_path_corridor(self) -> None:
        """Publish MPC corridor bounds as cross-track slats (LINE_LIST) in RViz2.

        Each slat is a line segment from the left boundary (ub_sm, magenta) to
        the right boundary (lb_sm, yellow) at each waypoint in the N-step horizon.
        Using LINE_LIST so every pair of points is one independent segment —
        this makes the pinch points immediately obvious without the lines
        clashing with the white track-boundary lines.
        """
        import math
        if self._mpc is None or not self._reference_path.waypoints:
            return
        wps = self._reference_path.waypoints
        N = int(self._mpc_cfg.N)
        start_id = self._mpc.model.wp_id
        n_wps = len(wps)

        left_pts: List[Point] = []
        right_pts: List[Point] = []

        for k in range(N):
            wp = wps[(start_id + k) % n_wps]
            if wp.ub_sm is None or wp.lb_sm is None:
                continue
            angle_ub = math.pi / 2 + wp.psi
            angle_lb = -math.pi / 2 + wp.psi
            # Left boundary (ub_sm = positive distance)
            lx = wp.x + float(wp.ub_sm) * math.cos(angle_ub)
            ly = wp.y + float(wp.ub_sm) * math.sin(angle_ub)
            # Right boundary (lb_sm = negative distance → use angle_lb for symmetry)
            rx = wp.x - float(wp.lb_sm) * math.cos(angle_lb)
            ry = wp.y - float(wp.lb_sm) * math.sin(angle_lb)
            left_pts.append(Point(x=lx, y=ly, z=0.3))
            right_pts.append(Point(x=rx, y=ry, z=0.3))

        # Build LINE_LIST: pairs (left[k], right[k]) = one transverse slat per waypoint
        slats = Marker()
        slats.header.frame_id = "map"
        slats.header.stamp = self.get_clock().now().to_msg()
        slats.ns = "mpc_corridor_slats"
        slats.id = 0
        slats.type = Marker.LINE_LIST
        slats.action = Marker.ADD
        slats.scale.x = 0.12   # line width in metres
        slats.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.85)  # bright yellow
        slats.pose.orientation.w = 1.0
        slats.lifetime.sec = 1
        for lp, rp in zip(left_pts, right_pts):
            slats.points.append(lp)  # type: ignore
            slats.points.append(rp)  # type: ignore

        # Left edge: magenta LINE_STRIP
        left_strip = Marker()
        left_strip.header.frame_id = "map"
        left_strip.header.stamp = self.get_clock().now().to_msg()
        left_strip.ns = "mpc_corridor_left"
        left_strip.id = 1
        left_strip.type = Marker.LINE_STRIP
        left_strip.action = Marker.ADD
        left_strip.scale.x = 0.15
        left_strip.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9)   # magenta = left
        left_strip.pose.orientation.w = 1.0
        left_strip.lifetime.sec = 1
        left_strip.points = left_pts  # type: ignore

        # Right edge: cyan LINE_STRIP
        right_strip = Marker()
        right_strip.header.frame_id = "map"
        right_strip.header.stamp = self.get_clock().now().to_msg()
        right_strip.ns = "mpc_corridor_right"
        right_strip.id = 2
        right_strip.type = Marker.LINE_STRIP
        right_strip.action = Marker.ADD
        right_strip.scale.x = 0.15
        right_strip.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)  # cyan = right
        right_strip.pose.orientation.w = 1.0
        right_strip.lifetime.sec = 1
        right_strip.points = right_pts  # type: ignore

        ma = MarkerArray()
        ma.markers = [slats, left_strip, right_strip]  # type: ignore
        self._corridor_pub.publish(ma)

    def _publish_dynamic_obstacles(self) -> None:
        """Publish dynamic V2X obstacle blobs as orange CYLINDER markers in RViz2."""
        ma = MarkerArray()
        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)  # type: ignore

        for i, ob in enumerate(self._dynamic_obstacles):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "v2x_obstacle_blobs"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(ob.cx)
            m.pose.position.y = float(ob.cy)
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = float(ob.radius) * 2
            m.scale.y = float(ob.radius) * 2
            m.scale.z = 1.0
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7)  # orange
            m.lifetime.sec = 1
            ma.markers.append(m)  # type: ignore

        self._dyn_obs_pub.publish(ma)

    def _publish_v2x_status(self) -> None:
        """Publish a floating text marker showing V2X gate state near the ego kart."""
        if self._odom is None or self._mpc is None:
            return
        ma = MarkerArray()
        txt = Marker()
        txt.header.frame_id = "map"
        txt.header.stamp = self.get_clock().now().to_msg()
        txt.ns = "v2x_status_text"
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = self._odom.pose.pose.position.x
        txt.pose.position.y = self._odom.pose.pose.position.y
        txt.pose.position.z = 4.0   # float 4 m above the kart
        txt.pose.orientation.w = 1.0
        txt.scale.z = 1.2
        txt.lifetime.sec = 1
        if self._v2x_avoidance_active:
            # Find nearest opponent ds for the status string
            ego_wp_id = self._mpc.model.wp_id
            ego_s = self._waypoint_s[min(ego_wp_id, len(self._waypoint_s) - 1)]
            min_ds = self._v2x_deactivation_dist + 1.0
            nearest_vid = "?"
            for v in getattr(self, '_last_v2x_msg', type('_Empty', (), {'vehicles': []})()).vehicles:
                opp_x, opp_y = float(v.position.x), float(v.position.y)
                dxy = self._waypoint_xy - np.array([opp_x, opp_y], dtype=np.float64)
                dists_sq = np.einsum('ij,ij->i', dxy, dxy)
                nearest_wp = int(np.argmin(dists_sq))
                if np.sqrt(dists_sq[nearest_wp]) < 0.8:
                    continue
                ds = (self._waypoint_s[nearest_wp] - ego_s) % self._total_track_s
                if ds < min_ds:
                    min_ds = ds
                    nearest_vid = v.vehicle_id
            gain = self._overtaking_steer_gain
            txt.text = f"V2X: ACTIVE | {nearest_vid} @ {min_ds:.0f}m | steer={gain:.1f}"
            txt.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)  # orange-red
        else:
            n_obs = len(self._dynamic_obstacles)
            state = self._recovery_state if hasattr(self, '_recovery_state') else "?"
            txt.text = f"V2X: monitoring | obs={n_obs} | {state}"
            txt.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)  # grey
        ma.markers.append(txt)  # type: ignore
        self._v2x_status_pub.publish(ma)

    def _publish_mpc_pred_marker(self, x_pred, y_pred):
        pred_marker_array = MarkerArray()
        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "mpc_pred"
        m_base.type = Marker.SPHERE
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale = Vector3(x=0.5, y=0.5, z=0.5)
        m_base.color = self._pred_marker_color
        for i in range(len(x_pred)):
            m = copy.deepcopy(m_base)
            m.id = i
            m.pose.position.x = x_pred[i]
            m.pose.position.y = y_pred[i]
            pred_marker_array.markers.append(m) # type: ignore
        self._mpc_pred_pub.publish(pred_marker_array)
        self._mpc_pred_pub_dummy.publish(pred_marker_array)

    def _publish_ref_path_marker(self, ref_path: ReferencePath):
        WP_SPHERE_ENABLED = False

        ref_path_marker_array = MarkerArray()

        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "ref_path"
        m_base.type = Marker.LINE_STRIP
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale.x = 0.2
        m_base.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)

        for i in range(len(ref_path.waypoints) - 1):
            m = copy.deepcopy(m_base)
            m.id = i
            start = Point()
            start.x = ref_path.waypoints[i].x
            start.y = ref_path.waypoints[i].y
            end = Point()
            end.x = ref_path.waypoints[i + 1].x
            end.y = ref_path.waypoints[i + 1].y
            m.points.append(start) # type: ignore
            m.points.append(end) # type: ignore
            ref_path_marker_array.markers.append(m) # type: ignore

        if WP_SPHERE_ENABLED:
            spheres = Marker()
            spheres.header.frame_id = "map"
            spheres.ns = "ref_path_point"
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            radius = 0.2
            spheres.scale = Vector3(x=radius, y=radius, z=radius)
            spheres.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            for i in range(len(ref_path.waypoints) - 1):
                p = Point()
                p.x = ref_path.waypoints[i].x
                p.y = ref_path.waypoints[i].y
                p.z = 0.
                spheres.points.append(p) #type: ignore
            ref_path_marker_array.markers.append(spheres) # type: ignore

        self._ref_path_pub.publish(ref_path_marker_array)
        self._ref_path_pub_dummy.publish(ref_path_marker_array)

    def _control(self):
        now = self.get_clock().now()
        t = (now - self._t_start).nanoseconds / 1e9
        dt = (now - self._last_t).nanoseconds / 1e9

        self._last_t = now
        self._loop += 1

        # record and print execution stats
        if self.use_stats:
            self._stats.record()

        # self.get_logger().info("loop")
        self._control_rate.sleep()

        if self._loop % 100 == 0:
            # update reference path
            if self._cfg.reference_path.update_by_topic: # type: ignore
                new_referece_path = self._create_reference_path_from_autoware_trajectory(self._trajectory)
                if new_referece_path is not None:
                    self._car.reference_path = new_referece_path
                    self._car.update_reference_path(self._car.reference_path)

            def plot_reference_path(car):
                import matplotlib.pyplot as plt
                import sys
                fig, ax = plt.subplots(1, 1)
                car.reference_path.show(ax)
                plt.show()
                sys.exit(1)
            # plot_reference_path(self._car)

        if self.USE_OBSTACLE_AVOIDANCE and self._obstacles_updated:
            self._obstacles_updated = False
            self._map.reset_map()
            if self._v2x_avoidance_active and self._recovery_state == "NORMAL":
                # Full obstacle avoidance: rasterize opponent predictions onto occupancy map
                filtered_dynamic = self._filter_obstacles_to_corridor(self._dynamic_obstacles)
                self._map.add_obstacles(self._static_obstacles + filtered_dynamic)
            else:
                # No active opponent or in recovery: use only static track boundaries
                # (prevents solver infeasibility from near-wall positions during recovery)
                self._map.add_obstacles(self._static_obstacles)
            self._reference_path.reset_dynamic_constraints()

        is_colliding = False
        if self._last_colliding_time is not None:
            elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
            if elapsed_from_last_colliding < 5.0:
                is_colliding = True

        pose = odom_to_pose_2d(self._odom) # type: ignore
        v = self._odom.twist.twist.linear.x

        self._car.update_states(pose.x, pose.y, pose.theta)
        # print(f"car x: {self._car.temporal_state.x}, y: {self._car.temporal_state.y}, psi: {self._car.temporal_state.psi}")
        # print(f"mpc x: {self._mpc.model.temporal_state.x}, y: {self._mpc.model.temporal_state.y}, psi: {self._mpc.model.temporal_state.psi}")

        if self._recovery_state == "RECOVERING":
            self._mpc_cfg.steering_tire_angle_gain_var = self._recovery_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._recovery_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self._recovery_state == "COOLDOWN":
            self._mpc_cfg.steering_tire_angle_gain_var = self._post_recovery_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._post_recovery_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self.USE_OBSTACLE_AVOIDANCE and self._v2x_avoidance_active:
            # Override per-waypoint steer gain with fixed overtaking gain while opponent is close
            self._mpc_cfg.steering_tire_angle_gain_var = self._overtaking_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._overtaking_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self._steer_gain_configurator is not None:
            current_gain = self._steer_gain_configurator.get_steer_gain(self._mpc.model.wp_id, interpolate=True)
            self._mpc_cfg.steering_tire_angle_gain_var = current_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / current_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)

        # --- Prediction marker color reflects current driving state ---
        # This works regardless of USE_BUG_ACC:
        #   RED    = actively recovering from wall (reversing)
        #   YELLOW = V2X overtaking mode is active (navigating around opponent)
        #   CYAN   = normal forward driving
        if self._dbg_pred_colors:
            if self._recovery_state == "RECOVERING":
                self._pred_marker_color = RED
            elif self.USE_OBSTACLE_AVOIDANCE and self._v2x_avoidance_active:
                self._pred_marker_color = YELLOW
            else:
                self._pred_marker_color = CYAN

        # Temporarily suppress obstacle avoidance inside the MPC solver when we are in
        # recovery, cooldown, or essentially stopped (|v| < 0.5 km/h). In those states
        # the kart is near a wall and update_path_constraints returns empty/infeasible
        # segments, which freeze the solver (u=[0,0]). Falling back to pre-computed
        # static bounds gives the same fast, stable behaviour as use_obstacle_avoidance=False.
        _suppress_obs_avoidance = (
            self.USE_OBSTACLE_AVOIDANCE and (
                self._recovery_state in ("RECOVERING", "COOLDOWN")
                or abs(v) < 0.5 / 3.6  # < 0.5 km/h  ≈ nearly stopped
            )
        )
        if _suppress_obs_avoidance:
            self._mpc.use_obstacle_avoidance = False

        with self._stats.time_block("control"):
            u, max_delta = self._mpc.get_control()
            # self.get_logger().info(f"u: {u}")

        if _suppress_obs_avoidance:
            self._mpc.use_obstacle_avoidance = True

        if self._mpc_cfg.use_csv_velocity:
            # We use the interpolated vx_mps from the CSV (stored in wp.v_ref)
            current_wp = self._reference_path.waypoints[self._mpc.model.wp_id]
            ref_vel_kmph = min(current_wp.v_ref * 3.6, self._mpc_cfg.v_max) # m/s to km/h
            self._mpc.update_v_max(ref_vel_kmph)
            # Note: We do NOT call set_v_ref to overwrite the whole path.
            # This allows the MPC to look ahead at the varying velocities in the CSV!
        else:
            if self._ref_vel_configulator is not None:
                ref_vel_mps = self._ref_vel_configulator.get_ref_vel(self._mpc.model.wp_id)
                ref_vel_kmph = min(
                    kmh_to_m_per_sec(ref_vel_mps),
                    self._mpc_cfg.v_max)
                self._mpc.update_v_max(ref_vel_kmph)
                v_ref: List[float] = [ref_vel_kmph] * len(self._reference_path.waypoints)
                self._reference_path.set_v_ref(v_ref)

        # override by brake command if control is disabled
        if not self._enable_control:
            last_v_cmd = self._last_u[0]
            if last_v_cmd < 0.5:
                u[0] = 0.0
            else:
                decel_v = last_v_cmd + self._mpc_cfg.a_min * dt
                u[0] = np.clip(decel_v, 0.0, self._mpc_cfg.v_max)

        if len(u) == 0:
            self.get_logger().error("No control signal", throttle_duration_sec=1)
            u = [0.0, 0.0]
            # continue

        # Wall Recovery State Machine
        is_recovering = False
        if self._recovery_enabled and self._enable_control:
            now_sec = t

            if self._recovery_state == "RECOVERING":
                if (now_sec - self._recovering_start_time) < self._recovery_reverse_time_sec:
                    is_recovering = True
                    # AWSIM: REVERSE gear + positive acc/speed moves the kart backward
                    u[0] = abs(self._recovery_reverse_vel)  # positive km/h target in REVERSE gear
                    # Negate steer: reverse opposite to forward path direction to back away from wall
                    u[1] = -u[1]
                    # Override steer gain to fixed recovery gain during recovery
                    self._mpc_cfg.steering_tire_angle_gain_var = self._recovery_steer_gain
                else:
                    self.get_logger().info("Wall recovery completed! Switching to DRIVE, resuming normal driving...")
                    self._recovery_state = "COOLDOWN"
                    self._cooldown_start_time = now_sec
                    self._stuck_start_time = None
                    self._recovering_start_time = None
                    self._last_u = np.array([0.0, 0.0])
                    self._last_acc = 0.0
                    self._car.update_reference_path(self._car.reference_path)
                    self._mpc_cfg.steering_tire_angle_gain_var = self._post_recovery_steer_gain
                    # Shift back to DRIVE gear so forward motion resumes
                    gear_msg = GearCommand()
                    gear_msg.stamp = self.get_clock().now().to_msg()
                    gear_msg.command = GearCommand.DRIVE
                    self._gear_pub.publish(gear_msg)

            elif self._recovery_state == "COOLDOWN":
                self._mpc_cfg.steering_tire_angle_gain_var = self._post_recovery_steer_gain
                if self._cooldown_start_time is not None and (now_sec - self._cooldown_start_time) >= self._recovery_cooldown_sec:
                    self._recovery_state = "NORMAL"
                    self._cooldown_start_time = None
                    self.get_logger().info("Recovery cooldown done. Back to NORMAL.")

            elif self._recovery_state == "NORMAL":
                curr_wp_id = self._mpc.model.wp_id
                wp_disabled = self._is_recovery_disabled_at_wp(curr_wp_id)

                if wp_disabled:
                    # In disabled waypoint range (e.g. race start / garage)
                    self._has_moved = False
                    self._stuck_start_time = None
                else:
                    # Track whether the kart has ever moved outside disabled zone
                    if abs(v) > self._recovery_stuck_vel_threshold:
                        self._has_moved = True

                    # u[0] is in km/h; v is in m/s. Stuck = forward commanded (u[0] > 3.6 km/h) but not moving.
                    # Only trigger after kart has actually moved at least once.
                    if self._has_moved and u[0] > 3.6 and abs(v) < self._recovery_stuck_vel_threshold:
                        if self._stuck_start_time is None:
                            self._stuck_start_time = now_sec
                            self.get_logger().warn("Stuck timer started...", throttle_duration_sec=1.0)
                        elif (now_sec - self._stuck_start_time) >= self._recovery_stuck_time_sec:
                            self.get_logger().warn(f"Stuck detected at wp {curr_wp_id}! Switching to REVERSE gear, recovering for {self._recovery_reverse_time_sec}s...")
                            self._recovery_state = "RECOVERING"
                            self._recovering_start_time = now_sec
                            is_recovering = True
                            u[0] = abs(self._recovery_reverse_vel)  # positive km/h in REVERSE gear
                            # Negate steer: reverse opposite to forward path direction to back away from wall
                            u[1] = -u[1]
                            self._mpc_cfg.steering_tire_angle_gain_var = self._recovery_steer_gain
                            # Shift to REVERSE gear
                            gear_msg = GearCommand()
                            gear_msg.stamp = self.get_clock().now().to_msg()
                            gear_msg.command = GearCommand.REVERSE
                            self._gear_pub.publish(gear_msg)
                    else:
                        self._stuck_start_time = None

        acc = 0.
        bug_acc_enabled = False
        if is_recovering:
            bug_acc_enabled = False
            # AWSIM: REVERSE gear + positive acc moves backward
            acc = self._mpc_cfg.a_max  # full forward acc in REVERSE gear = move backward
        elif self.USE_BUG_ACC:
            def deg2rad(deg):
                return deg * np.pi / 180.0

            if abs(v) > kmh_to_m_per_sec(44.0) or \
             (abs(v) > kmh_to_m_per_sec(38.0) and abs(max_delta) > deg2rad(12.0)):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_min / 3.0 * 2.0
                self._pred_marker_color = RED
            elif abs(v) > kmh_to_m_per_sec(41.0) or abs(u[1]) > deg2rad(10.0):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_max
                self._pred_marker_color = YELLOW
            else:
                bug_acc_enabled = True
                acc = 500.0
                self._pred_marker_color = CYAN
        else:
            acc =  self.KP * (u[0] - v)
            # print(f"v: {v}, u[0]: {u[0]}, acc: {acc}")
            acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)
        # u[0] = np.clip(last_u[0] + acc * dt, 0.0, self._mpc_cfg.v_max)

        # apply low pass filter to control signal
        acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
        u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain

        self._last_acc = acc
        self._last_u[0] = u[0]
        self._last_u[1] = u[1]

        # update car state (use v for feedback actual speed)
        self._car.drive([v, u[1]])

        # Publish control command
        self._publish_control_command(now, u, acc, bug_acc_enabled)

        # Log states
        self._sim_logger.log(self._car, u, t)
        self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)

        # 約 0.25 秒ごとに予測結果を表示
        if (self._mpc.current_prediction is not None) and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
            self._publish_mpc_pred_marker(self._mpc.current_prediction[0], self._mpc.current_prediction[1]) # type: ignore
            if self.USE_OBSTACLE_AVOIDANCE:
                if self._dbg_corridor:
                    self._publish_path_corridor()
                if self._dbg_dyn_obstacles:
                    self._publish_dynamic_obstacles()
                if self._dbg_v2x_status_text:
                    self._publish_v2x_status()

    def run(self) -> None:
        self._wait_until_clock_received()
        self._wait_until_odom_received()
        self._wait_until_trajectory_received()
        self._wait_until_path_constraints_received()

        # initialize car states
        pose = odom_to_pose_2d(self._odom) # type: ignore
        self._car.update_states(pose.x, pose.y, pose.theta)
        self._car.update_reference_path(self._car.reference_path)

        # Always publish ref path at startup — latching QoS (TRANSIENT_LOCAL) delivers
        # it to RViz2 even if RViz2 subscribes after the node starts.
        # Previously this was gated on `_ref_vel_configulator is None`, which broke it
        # when use_csv_velocity=false.
        self._publish_ref_path_marker(self._car.reference_path)

        self._pred_marker_color = CYAN

        # for i in range(10):
        #     self._obstacle_manager.push_next_obstacle()

        # initialize control states
        self._control_rate = self.create_rate(self._mpc_cfg.control_rate)
        self._sim_logger = SimulationLogger(
            self.get_logger(),
            self._car.temporal_state.x, self._car.temporal_state.y, self._cfg.sim_logger.animation_enabled, self.SHOW_PLOT_ANIMATION, self.PLOT_RESULTS, self.ANIMATION_INTERVAL) # type: ignore

        self._loop = 0
        self._last_acc = 0.0
        self._last_u = np.array([0.0, 0.0])
        self._t_start = self.get_clock().now()
        self._last_t = self._t_start

        self.get_logger().info("----------------------")
        self.get_logger().info("START!")
        self.get_logger().info("----------------------")

        while rclpy.ok() and (not self._sim_logger.stop_requested()):
            self._control()

    def stop(self):
        # Wait for stopping
        self.get_logger().warn("----------------------")
        self.get_logger().warn("Stopping...")
        self.get_logger().warn("----------------------")
        timeout_time = self.get_clock().now() + rclpy.time.Duration(seconds=5)
        while self._odom.twist.twist.linear.x > 0.1 and self.get_clock().now() < timeout_time:
            self._enable_control = False
            self._control()

        # Publish zero command to stop the car completely
        zero_cmd = self._create_ackerman_control_command(self.get_clock().now(), [0.0, 0.0], 0.0, False)
        self._command_pub.publish(zero_cmd)

        self.get_logger().warn(">> Stop Completed!")

        # show results
        self._sim_logger.show_results(self._current_laps, self._lap_times, self._car)

    @classmethod
    def in_pkg_share(cls, file_path: str) -> str:
        return cls.PKG_PATH + file_path
