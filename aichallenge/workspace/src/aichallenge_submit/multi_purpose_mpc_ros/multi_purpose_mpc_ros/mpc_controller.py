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
from multi_purpose_mpc_ros.core.opponent_profiler import OpponentProfiler
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

    def _read_config_file(self, file_path: str) -> str:
        if file_path.endswith('.enc'):
            import base64
            import zlib
            with open(file_path, "rb") as f:
                content_bytes = f.read()
            return zlib.decompress(base64.b85decode(content_bytes)).decode('utf-8')
        else:
            with open(file_path, "r") as f:
                return f.read()

    def _load_config(self) -> NamedTuple:
        config_content = self._read_config_file(self._config_path)
        if not self._config_path.endswith('.enc'):
            self.get_logger().info("\n----- config.yaml -----\n" + config_content + "\n-----------------------")
        else:
            self.get_logger().info("\n----- config (encrypted) loaded -----")

        if self._ref_vel_config_path is not None:
            ref_vel_config_content = self._read_config_file(self._ref_vel_config_path)
            if not self._ref_vel_config_path.endswith('.enc'):
                self.get_logger().info("\n----- ref_vel.yaml -----\n" + ref_vel_config_content + "\n-----------------------")
            else:
                self.get_logger().info("\n----- ref_vel (encrypted) loaded -----")

        if self._steer_gain_config_path is not None:
            steer_gain_config_content = self._read_config_file(self._steer_gain_config_path)
            if not self._steer_gain_config_path.endswith('.enc'):
                self.get_logger().info("\n----- steer_gain.yaml -----\n" + steer_gain_config_content + "\n-----------------------")
            else:
                self.get_logger().info("\n----- steer_gain (encrypted) loaded -----")

        cfg: NamedTuple = convert_to_namedtuple(yaml.safe_load(config_content)) # type: ignore

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
            self.declare_parameter("bicycle_model_width", self._cfg.bicycle_model.width)

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
                    mpc_cfg.v_max = kmh_to_m_per_sec(param.value)
                    self._mpc.update_v_max(mpc_cfg.v_max)
                    v_ref: List[float] = [mpc_cfg.v_max] * len(self._reference_path.waypoints)
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

                elif param.name in ("bicycle_model_width", "bicycle_model.width") and param.type_ == Parameter.Type.DOUBLE:
                    self._cfg.bicycle_model.width = param.value
                    if hasattr(self, '_car') and self._car is not None:
                        self._car.width = param.value
                    if hasattr(self, '_mpc') and self._mpc is not None and hasattr(self._mpc, 'model') and self._mpc.model is not None:
                        self._mpc.model.width = param.value
                    self.get_logger().warn(f"bicycle_model.width was updated to '{param.value}' [m]")


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

        def create_mpc(car: BicycleModel, override_N=None) -> Tuple[MPCConfig, MPC]:
            cfg_mpc = self._cfg.mpc # type: ignore
            N_to_use = override_N if override_N is not None else cfg_mpc.N

            mpc_cfg = MPCConfig(
                N_to_use,
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
                N_to_use,
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
        self._create_mpc = create_mpc # bind to self for hot-swapping N later
        self._mpc_cfg, self._mpc = self._create_mpc(self._car)
        compute_speed_profile(self._car, self._mpc_cfg)
        # Snapshot the baseline steer gain and default N from config so they can be restored
        # after overtaking or recovery overrides are released.
        self._baseline_steer_gain = self._mpc_cfg.steering_tire_angle_gain_var
        self._default_mpc_N = int(self._mpc_cfg.N)

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
        
        # Unconditionally initialize _waypoint_xy for prediction marker publishing
        wps = self._reference_path.waypoints
        self._waypoint_xy = np.asarray([(wp.x, wp.y) for wp in wps], dtype=np.float64)
        
        self._opponent_profiler = None

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            if self._static_obstacles:
                self._map.add_obstacles(self._static_obstacles)
            self._map.save_static_map()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = False
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )
            
            # Opponent Profiler
            self._opponent_profiler = OpponentProfiler(wps, logger=self.get_logger())
            
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)  # type: ignore
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(0, mpc_N, 4)
            ]
            # コリドー外の V2X 障害物で MPC のコリドー狭窄/反転が起きないよう、
            # ref-path 近傍のみに絞り込む。閾値 = max_width/2 + vehicle_radius + 余白。
            ref_max_width = float(self._cfg.reference_path.max_width)  # type: ignore
            self._v2x_corridor_threshold_sq = (
                ref_max_width / 2.0 + self._v2x_vehicle_radius + 0.5
            ) ** 2

            # Build cumulative arc-length array for Frenet-frame proximity check
            diffs = np.diff(self._waypoint_xy, axis=0)
            seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
            self._waypoint_s = np.concatenate([[0.0], np.cumsum(seg_lengths)])
            self._total_track_s = float(self._waypoint_s[-1])
            # Proximity gate params (with hysteresis)
            self._v2x_activation_dist = float(v2x_cfg.activation_distance_m)  # type: ignore
            self._v2x_deactivation_dist = float(v2x_cfg.deactivation_distance_m)  # type: ignore
            self._min_overtaking_steer_gain = 1.2 # Default, updated dynamically

            # Tailing & Overtaking Config
            self._v2x_tailing_distance = float(getattr(v2x_cfg, 'tailing_distance_m', 15.0))
            
            # Strict Tailing Config (Real No-Overtake Zones)
            self._strict_tailing_speed_gain = float(getattr(v2x_cfg, 'strict_tailing_speed_gain', 0.1))
            self._strict_tailing_min_delta = float(getattr(v2x_cfg, 'strict_tailing_min_delta', 0.5))
            self._strict_no_overtake_zones = getattr(v2x_cfg, 'strict_no_overtake_zones', [])
            
            self._corridor_bias = [] # Defaults to empty, set dynamically based on opponent
            self._fast_opponent_cfg = getattr(v2x_cfg, 'fast_opponent', None)
            self._slow_opponent_cfg = getattr(v2x_cfg, 'slow_opponent', None)
            
            # Default fallback if config is missing
            self._v2x_fast_threshold = float(getattr(self._fast_opponent_cfg, 'threshold_kmh', 18.0)) if self._fast_opponent_cfg else 18.0
            # Runtime state
            self._v2x_avoidance_active = False
            self._is_launch_mode_active = False
            self._prev_launch_mode_active = False
            # Minimum time (seconds) the V2X active state must hold before it can flip again.
            # Set to 1.0s to prevent premature deactivation when ego is stopped near opponent.
            self._v2x_min_dwell_sec = 1.0
            self._v2x_last_state_change_time = None

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
            self._recovery_cooldown_Q0 = float(getattr(recovery_cfg, 'cooldown_Q0', 10000000.0))
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
            self._recovery_cooldown_Q0 = 10000000.0
            raw_disabled = None

        self._default_Q0 = float(self._cfg.mpc.Q[0])

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
        self._node_start_time = None
        self._recovering_start_time = None
        self._cooldown_start_time = None
        # Accumulated post-recovery steer gain counters.
        # _recovery_cycle_count increments each time a full stuck cycle completes
        # (RECOVERING → COOLDOWN).  _current_post_recovery_steer_gain tracks the
        # effective (possibly elevated) gain for the current/next cooldown phase.
        self._recovery_cycle_count = 0
        self._current_post_recovery_steer_gain = self._post_recovery_steer_gain
        self._post_recovery_overtake_pending = False
        self._overtaking_cycle_count = 0
        self._overtaking_steer_gain = 1.2
        self._overtaking_gain_increment = 0.5
        self._overtaking_gain_max = 3.0
        self._current_overtaking_steer_gain = 1.2
        # Only allow stuck detection after the kart has moved at least once
        self._has_moved = False

        # save config
        if self._cfg.common.save_config:
            self._save_config()

    @staticmethod
    def _interpolate_gain(gain_config, current_v_kmh, default_gain=1.2):
        if isinstance(gain_config, list) and len(gain_config) > 0:
            try:
                # Expecting [[speed1, gain1], [speed2, gain2], ...]
                pts = sorted(gain_config, key=lambda x: x[0])
                if current_v_kmh <= pts[0][0]:
                    return float(pts[0][1])
                if current_v_kmh >= pts[-1][0]:
                    return float(pts[-1][1])
                # Interpolate
                for i in range(len(pts) - 1):
                    if pts[i][0] <= current_v_kmh <= pts[i+1][0]:
                        v1, g1 = float(pts[i][0]), float(pts[i][1])
                        v2, g2 = float(pts[i+1][0]), float(pts[i+1][1])
                        ratio = (current_v_kmh - v1) / (v2 - v1) if v2 > v1 else 0.0
                        return float(g1 + ratio * (g2 - g1))
            except Exception:
                pass
            return default_gain
        else:
            try:
                return float(gain_config)
            except (TypeError, ValueError):
                return default_gain

    def _set_mpc_horizon(self, new_N: int):
        if self._mpc_cfg is not None and int(self._mpc_cfg.N) == int(new_N):
            return # Already set
            
        self.get_logger().info(f"[MPC] Hot-swapping prediction horizon (N): {int(self._mpc_cfg.N)} -> {int(new_N)}")
        
        # 1. Save previous steering state to maintain continuity
        prev_steer = self._mpc.previous_steering if self._mpc is not None else 0.0
        
        # 2. Reconstruct MPC with override_N
        self._mpc_cfg, self._mpc = self._create_mpc(self._car, override_N=new_N)
        self._mpc.previous_steering = prev_steer
        
        # 4. Re-calculate v2x_t_samples which depend on N
        if hasattr(self, '_v2x_t_samples'):
            mpc_N = int(new_N)
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(0, mpc_N, 4)
            ]
        
        # 5. Pre-build simple path constraints since N changed
        if self.USE_OBSTACLE_AVOIDANCE:
            self._reference_path.update_simple_path_constraints(
                int(self._mpc_cfg.N),
                self._car.safety_margin)

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
        self._pure_pursuit_pub = self.create_publisher(
            Marker,
            "/pure_pursuit_target",
            1
        )
        
        from sensor_msgs.msg import LaserScan
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "/sensing/lidar/scan", self._scan_callback, scan_qos)
        self._last_scan = None
        self._scan_marker_pub = self.create_publisher(Marker, "lidar_points", 1)

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

    def _cmd_callback(self, msg: AckermannControlCommand):
        self._current_cmd = msg
        
    def _scan_callback(self, msg):
        self._last_scan = msg
        
        # Publish the scan as points so the user can visualize it easily
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        import math
        
        marker = Marker()
        marker.header = msg.header
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        # Small green dots
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        ranges = msg.ranges
        for i, r in enumerate(ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                p = Point()
                p.x = float(r * math.cos(angle))
                p.y = float(r * math.sin(angle))
                p.z = 0.0
                marker.points.append(p)
                
        self._scan_marker_pub.publish(marker)

    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _control_mode_request_callback(self, msg):
        if msg.data and not self._enable_control:
            self.get_logger().info("Control mode request received")
            self._enable_control = True
            self._has_moved = False
            self._recovery_state = "NORMAL"
            if hasattr(self, '_default_Q0'):
                self._set_Q0(self._default_Q0)
            self._stuck_start_time = None
        elif not msg.data and self._enable_control:
            self._enable_control = False
            self._has_moved = False
            self._recovery_state = "NORMAL"
            if hasattr(self, '_default_Q0'):
                self._set_Q0(self._default_Q0)
            self._stuck_start_time = None

    def _set_Q0(self, q0_value: float):
        if self._cfg.mpc.Q[0] == q0_value:
            return
        self._cfg.mpc.Q[0] = q0_value
        if hasattr(self, '_mpc_cfg') and hasattr(self, '_mpc'):
            self._mpc_cfg.Q = sparse.diags(self._cfg.mpc.Q)
            self._mpc.update_Q(self._mpc_cfg.Q)
            self.get_logger().info(f"[RECOVERY DEBUG] MPC Q[0] dynamically updated to {q0_value}")

    def _path_constraints_callback(self, msg: PathConstraints):
        self._reference_path.set_path_constraints(
            msg.upper_bounds, msg.lower_bounds, msg.rows, msg.cols)

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        if not hasattr(self, '_waypoint_s'):
            return
            
        import time
        t_v2x_start = time.time()
        self._v2x_tracker.update(msg)
        
        if not hasattr(self, '_last_v2x_print_time'):
            self._last_v2x_print_time = 0.0

        now_wall = self.get_clock().now().nanoseconds / 1e9
        should_print = (now_wall - self._last_v2x_print_time) > 1.0

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
        
        ego_x = self._car.temporal_state.x if self._car else 0.0
        ego_y = self._car.temporal_state.y if self._car else 0.0


        min_opponent_ds = float('inf')
        nearest_opponent_vid = None

        parsed_vehicles = []
        for idx, v in enumerate(msg.vehicles):
            opp_x = float(v.position.x)
            opp_y = float(v.position.y)
            
            # Check if this vehicle is actually ego
            dist_to_ego = np.hypot(opp_x - ego_x, opp_y - ego_y)
            if dist_to_ego < 0.5:  # 0.5m threshold to filter out self
                continue
                
            # Snap opponent to nearest waypoint
            dxy = self._waypoint_xy - np.array([opp_x, opp_y], dtype=np.float64)
            dists_sq = np.einsum('ij,ij->i', dxy, dxy)
            nearest_wp_id = int(np.argmin(dists_sq))
            opp_s = self._waypoint_s[nearest_wp_id]
            parsed_vehicles.append({
                'vid': v.vehicle_id, 'x': opp_x, 'y': opp_y, 's': opp_s, 'wp': nearest_wp_id, 'v': v
            })

        for p_opp in parsed_vehicles:
            v = p_opp['v']
            opp_s = p_opp['s']
            nearest_wp_id = p_opp['wp']
            opp_x = p_opp['x']
            opp_y = p_opp['y']
            
            if self._opponent_profiler:
                # Check if this opponent is clear (no one within 15m AHEAD of them)
                # If they are 15m behind someone, they are overtaking, don't learn!
                is_clear = True
                
                # Check ego
                ds_ego = (ego_s - opp_s) % track_len
                if 0 < ds_ego < 15.0:
                    is_clear = False
                    
                if is_clear:
                    for other in parsed_vehicles:
                        if other['vid'] == v.vehicle_id:
                            continue
                        ds_other = (other['s'] - opp_s) % track_len
                        if 0 < ds_other < 15.0:
                            is_clear = False
                            break
                            
                if is_clear:
                    heading = self._v2x_tracker.heading(v.vehicle_id)
                    self._opponent_profiler.record_opponent_at_wp(opp_x, opp_y, v.vehicle_id, nearest_wp_id, opp_yaw=heading)
                
            if should_print:
                vel = self._v2x_tracker.velocity(v.vehicle_id)
                acc = self._v2x_tracker.acceleration(v.vehicle_id)
                heading = self._v2x_tracker.heading(v.vehicle_id)
                import math
                v_mag = math.hypot(vel[0], vel[1])
                a_mag = math.hypot(acc[0], acc[1])
                self.get_logger().info(f"[STATE ESTIMATION] {v.vehicle_id} tracked -> Vel: {v_mag:.2f} m/s, Acc: {a_mag:.2f} m/s^2, Heading: {math.degrees(heading):.1f} deg")
            
            # Signed forward distance along raceline; wraps cleanly on circular track
            ds = (opp_s - ego_s) % track_len

            # ds in (0, track_len): small = just ahead, large = behind us
            # Allow opponent to be slightly behind us (e.g. ds > track_len - 2.0) and still keep avoidance active
            if 0.0 <= ds <= activate_threshold or ds > track_len - 2.0:
                any_opponent_in_activate_range = True
            if 0.0 <= ds <= deactivate_threshold or ds > track_len - 3.0:
                any_opponent_in_deactivate_range = True
                
                # Track the nearest opponent for speed matching (tailing)
                actual_ds = ds if ds <= deactivate_threshold else ds - track_len
                if actual_ds > 0 and actual_ds < min_opponent_ds:
                    min_opponent_ds = actual_ds
                    nearest_opponent_vid = v.vehicle_id
                    
        self._nearest_opponent_vid = nearest_opponent_vid
        self._nearest_opponent_ds = min_opponent_ds

        # Apply hysteresis: activate on close approach, deactivate only when all clear.
        # Additionally enforce a minimum dwell time to prevent wp_id jitter from toggling
        # the state multiple times per second.
        prev_active = self._v2x_avoidance_active
        now_wall = self.get_clock().now().nanoseconds / 1e9
        dwell_elapsed = (
            (now_wall - self._v2x_last_state_change_time) >= self._v2x_min_dwell_sec
            if self._v2x_last_state_change_time is not None else True
        )
        desired_active = prev_active  # default: hold current
        if any_opponent_in_activate_range:
            desired_active = True
        elif not any_opponent_in_deactivate_range:
            desired_active = False
        # Only flip if the minimum dwell period has elapsed
        if desired_active != prev_active and dwell_elapsed:
            self._v2x_avoidance_active = desired_active
            self._v2x_last_state_change_time = now_wall

        if self._v2x_avoidance_active != prev_active:
            if self._v2x_avoidance_active:
                # Check if this activation follows a recovery — if so, accumulate the overtaking gain
                if self._post_recovery_overtake_pending:
                    self._post_recovery_overtake_pending = False
                    self._overtaking_cycle_count += 1
                    accumulated = self._overtaking_steer_gain + \
                        self._overtaking_cycle_count * self._overtaking_gain_increment
                    if self._overtaking_gain_max is not None:
                        accumulated = min(accumulated, self._overtaking_gain_max)
                    self._current_overtaking_steer_gain = accumulated
                    self.get_logger().warn(
                        f"[V2X] Overtaking cycle #{self._overtaking_cycle_count}: "
                        f"overtaking_steer_gain -> {self._current_overtaking_steer_gain:.2f}")
                self.get_logger().warn(
                    "[V2X] Overtaking mode ACTIVATED — opponent within "
                    f"{activate_threshold:.0f} m ahead.")
            else:
                self.get_logger().info(
                    "[V2X] Overtaking mode DEACTIVATED — no opponent within "
                    f"{deactivate_threshold:.0f} m ahead. Resuming normal steer gain.")

        # Store last message for _publish_v2x_status (needs vehicle IDs)
        self._last_v2x_msg = msg

        if self._v2x_avoidance_active:
            predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
            new_dynamic = predictions_to_obstacles(predictions, self._v2x_vehicle_radius)
            
            # Populate Frenet s for each obstacle to prevent ghost projection bugs in sharp curves
            if self._waypoint_xy.size > 0:
                printed_strategy_vids = set()
                for ob in new_dynamic:
                    dxy = self._waypoint_xy - np.array([ob.cx, ob.cy], dtype=np.float64)
                    dists_sq = np.einsum('ij,ij->i', dxy, dxy)
                    nearest_wp = int(np.argmin(dists_sq))
                    ob.s = self._waypoint_s[nearest_wp]
                    ob.nearest_wp_id = nearest_wp
                    if self._opponent_profiler:
                        ob.strategy = self._opponent_profiler.get_strategy_for_corner(ob.vid, nearest_wp)
                        if should_print:
                            # Only log for the nearest opponent to reduce noise, and only once (since there are multiple prediction points)
                            if getattr(self, '_nearest_opponent_vid', None) == ob.vid and ob.vid not in printed_strategy_vids:
                                self.get_logger().info(f"[STRATEGY] Opponent '{ob.vid}' projected at wp {nearest_wp}. Applying '{ob.strategy}' corridor strategy.")
                                printed_strategy_vids.add(ob.vid)

            # Only trigger a map rebuild if obstacle positions changed meaningfully (>0.1m).
            # This prevents continuous corridor redraw on every V2X callback when the
            # opponent is stationary or barely moving (e.g. ego kart stopped in manual mode).
            if self._obstacles_changed(new_dynamic):
                self._dynamic_obstacles = new_dynamic
        else:
            # Opponent left range — clear dynamic obstacles so map resets to clean static corridor
            if self._dynamic_obstacles:
                self._dynamic_obstacles = []

        # --- V2X diagnostic terminal log (throttled, enabled via config) ---
        if self._dbg_v2x_log:
            self._v2x_diagnostic_log(
                msg, ego_wp_id, ego_s, track_len,
                activate_threshold, deactivate_threshold)
                
        if should_print:
            self._last_v2x_print_time = now_wall

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

    def _obstacles_changed(self, new_obstacles: List[Obstacle], threshold: float = 0.1) -> bool:
        """Return True if the new obstacle list differs meaningfully from the current one.

        Compares list length and, if equal, the max displacement of any obstacle.
        Prevents continuous map rebuilds when the opponent is stationary or barely moving.
        """
        old = self._dynamic_obstacles
        if len(new_obstacles) != len(old):
            return True
        if not new_obstacles:
            return False
        for o_new, o_old in zip(new_obstacles, old):
            if (o_new.cx - o_old.cx) ** 2 + (o_new.cy - o_old.cy) ** 2 > threshold ** 2:
                return True
        return False

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

    def _compute_corridor_width_at_ego(self) -> float:
        """Return the minimum corridor width (ub - lb) across the current MPC horizon
        at the ego waypoint, reading dynamic path constraints directly.
        This is independent of the MPC solver's use_obstacle_avoidance flag so it
        works correctly even when the solver has OA suppressed (e.g. while stopped)."""
        try:
            pc = self._reference_path.path_constraints
            if pc is None or len(pc) < 2:
                return float('inf')  # no constraints yet — safe fallback
            ub_all = pc[0]  # shape: (n_waypoints, N)
            lb_all = pc[1]
            wp_id = (self._mpc.model.wp_id + 1) % len(ub_all)
            ub_row = ub_all[wp_id]  # array of length N
            lb_row = lb_all[wp_id]
            widths = ub_row - lb_row
            return float(np.min(widths))  # most restrictive point in the horizon
        except Exception:
            return float('inf')  # safe fallback: do not trigger stop

    # =========================================================================
    # Debug Visualization Helpers
    # =========================================================================

    def _v2x_diagnostic_log(self, msg, ego_wp_id, ego_s, track_len,
                             activate_threshold, deactivate_threshold) -> None:
        """Throttled terminal log: prints V2X vehicle positions/distances and gate state."""
        if getattr(self, '_recovery_state', "NORMAL") != "NORMAL":
            return
            
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
            ego_x = float(self._odom.pose.pose.position.x) if self._odom is not None else 0.0
            ego_y = float(self._odom.pose.pose.position.y) if self._odom is not None else 0.0
            dist_to_ego = np.hypot(opp_x - ego_x, opp_y - ego_y)
            is_ego = (dist_to_ego < 0.5)
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

        Both boundary strips always begin at the ego kart's world position so
        the corridor is visually anchored to the car even when an obstacle is
        very close ahead (which would otherwise leave a gap between the kart
        and the first rendered waypoint).
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

        # Use the waypoint nearest to the ego's actual world position as start,
        # not mpc.model.wp_id which can jump forward past obstacles when the
        # solver is confused, causing the corridor to skip the blocking section.
        if self._odom is not None:
            ego_x = self._odom.pose.pose.position.x
            ego_y = self._odom.pose.pose.position.y
            dxy = self._waypoint_xy - np.array([ego_x, ego_y], dtype=np.float64)
            start_id = int(np.argmin(np.einsum('ij,ij->i', dxy, dxy)))
        # else keep start_id = mpc.model.wp_id (set above)

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
                
                # Check if this vehicle is actually ego
                ego_x = self._car.temporal_state.x if self._car else 0.0
                ego_y = self._car.temporal_state.y if self._car else 0.0
                if np.hypot(opp_x - ego_x, opp_y - ego_y) < 0.5:
                    continue
                    
                dxy = self._waypoint_xy - np.array([opp_x, opp_y], dtype=np.float64)
                dists_sq = np.einsum('ij,ij->i', dxy, dxy)
                nearest_wp = int(np.argmin(dists_sq))
                ds = (self._waypoint_s[nearest_wp] - ego_s) % self._total_track_s
                if ds < min_ds:
                    min_ds = ds
                    nearest_vid = v.vehicle_id
            gain = self._mpc_cfg.steering_tire_angle_gain_var
            txt.text = f"V2X: ACTIVE | {nearest_vid} @ {min_ds:.0f}m | steer={gain:.1f}"
            txt.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)  # orange-red
        else:
            n_obs = len(self._dynamic_obstacles)
            state = self._recovery_state if hasattr(self, '_recovery_state') else "?"
            txt.text = f"V2X: monitoring | obs={n_obs} | {state}"
            txt.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)  # grey
        ma.markers.append(txt)  # type: ignore
        self._v2x_status_pub.publish(ma)

    def _publish_mpc_pred_marker(self, spatial_state_prediction):
        if self._odom is None or self._mpc is None:
            return

        # Use the waypoint nearest to the ego's actual world position as start,
        # preventing the prediction visual from jumping over the opponent
        ego_x = self._odom.pose.pose.position.x
        ego_y = self._odom.pose.pose.position.y
        dxy = self._waypoint_xy - np.array([ego_x, ego_y], dtype=np.float64)
        start_id = int(np.argmin(np.einsum('ij,ij->i', dxy, dxy)))

        N = min(self._mpc_cfg.N, len(spatial_state_prediction))
        
        x_pred, y_pred = [], []
        # First state is current state (n=0, 1 are handled implicitly in solver, but prediction array starts at n=0)
        # N steps means n ranges from 0 to N. The original loop did range(2, N). We match that.
        for n in range(2, N):
            associated_waypoint = self._mpc.model.reference_path.get_waypoint(start_id + n)
            predicted_temporal_state = self._mpc.model.s2t(associated_waypoint, spatial_state_prediction[n, :])
            x_pred.append(predicted_temporal_state.x)
            y_pred.append(predicted_temporal_state.y)

        pred_marker_array = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mpc_pred"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=0.5, y=0.5, z=0.5)
        m.color = self._pred_marker_color
        
        for i in range(len(x_pred)):
            p = Point()
            p.x = x_pred[i]
            p.y = y_pred[i]
            p.z = 0.0
            m.points.append(p) # type: ignore
            
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
        now_sec = now.nanoseconds / 1e9
        
        if self._node_start_time is None:
            self._node_start_time = now_sec
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

        if hasattr(self, '_mpc') and hasattr(self, '_corridor_bias'):
            self._mpc.model.reference_path.corridor_bias = self._corridor_bias

        if self.USE_OBSTACLE_AVOIDANCE:
            self._reference_path.reset_dynamic_constraints()

        is_colliding = False
        if self._last_colliding_time is not None:
            elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
            if elapsed_from_last_colliding < 5.0:
                is_colliding = True

        pose = odom_to_pose_2d(self._odom) # type: ignore
        v = self._odom.twist.twist.linear.x

        self._car.update_states(pose.x, pose.y, pose.theta)
        
        ego_wp_id = self._mpc.model.wp_id if hasattr(self, '_mpc') and self._mpc is not None else 0
        
        if not hasattr(self, '_ego_laps'):
            self._ego_laps = 0
            self._last_ego_wp = ego_wp_id
            
        total_wps = len(self._reference_path.waypoints)
        if self._last_ego_wp > total_wps * 0.8 and ego_wp_id < total_wps * 0.2:
            self._ego_laps += 1
            self.get_logger().info(f"[RACE] Lap {self._ego_laps} completed!")
        elif abs(ego_wp_id - self._last_ego_wp) > 10:
            self.get_logger().warn(f"[TELEPORT DETECTED] wp_id jumped from {self._last_ego_wp} to {ego_wp_id}. Resetting all states!")
            self._recovery_state = "NORMAL"
            self._stuck_start_time = None
            self._recovering_start_time = None
            self._cooldown_start_time = None
            self._enable_control = True
            self._ego_laps = 0
            self._is_active_hard_stop = False
            self._is_launch_mode_active = False
            self._in_alley_exit_mode = False
            self._alley_exit_finished = False
            # Reset V2X hysteresis timers so it can activate instantly at the new gate
            self._v2x_last_state_change_time = None
        self._last_ego_wp = ego_wp_id
        
        # === START LINE LAUNCH OVERRIDE ===
        # At the start of the race, everyone is packed together and starting from 0 km/h.
        # Tailing logic can cause a deadlock. Force a launch to slam the gas and break out!
        self._is_start_line_override = False # Temporarily disabled: (self._ego_laps == 0 and ego_wp_id < 40)
        # print(f"car x: {self._car.temporal_state.x}, y: {self._car.temporal_state.y}, psi: {self._car.temporal_state.psi}")
        # print(f"mpc x: {self._mpc.model.temporal_state.x}, y: {self._mpc.model.temporal_state.y}, psi: {self._mpc.model.temporal_state.psi}")

        # V2X overtaking steer gain takes priority ALWAYS, even during recovery/cooldown,
        # so that overtaking corridor calculations are never disrupted by wall recovery mode.
        # NOTE: We only apply the min_overtaking_steer_gain when LAUNCH MODE is active!
        if self.USE_OBSTACLE_AVOIDANCE and self._is_launch_mode_active:
            if self._steer_gain_configurator is not None:
                current_gain = self._steer_gain_configurator.get_steer_gain(self._mpc.model.wp_id, interpolate=True)
            else:
                current_gain = self._mpc_cfg.steering_tire_angle_gain_var
            
            overtaking_gain = max(current_gain, self._min_overtaking_steer_gain)
            self._mpc_cfg.steering_tire_angle_gain_var = overtaking_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / overtaking_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self._recovery_state == "RECOVERING":
            self._mpc_cfg.steering_tire_angle_gain_var = self._recovery_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._recovery_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self._recovery_state == "COOLDOWN":
            self._mpc_cfg.steering_tire_angle_gain_var = self._current_post_recovery_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._current_post_recovery_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        elif self._steer_gain_configurator is not None:
            # Normal driving with per-waypoint dynamic gain
            current_gain = self._steer_gain_configurator.get_steer_gain(self._mpc.model.wp_id, interpolate=True)
            self._mpc_cfg.steering_tire_angle_gain_var = current_gain
            safe_gain = max(current_gain, 1e-6)
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / safe_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)
        else:
            # Normal driving, fixed gain — always restore baseline so any previous
            # overtaking/recovery override (e.g. overtaking_steer_gain) is cleared.
            if self._mpc_cfg.steering_tire_angle_gain_var != self._baseline_steer_gain:
                self.get_logger().info(
                    f"[SteerGain] Restoring baseline gain "
                    f"{self._mpc_cfg.steering_tire_angle_gain_var:.3f} -> {self._baseline_steer_gain:.3f}")
            self._mpc_cfg.steering_tire_angle_gain_var = self._baseline_steer_gain
            scaled_steer_rate_max = self._mpc_cfg.steer_rate_max / self._baseline_steer_gain
            self._mpc.update_max_steering_rate(scaled_steer_rate_max)

        # --- Prediction marker color reflects current driving state ---
        # V2X (YELLOW) always takes display priority over recovery (RED).
        #   RED    = actively recovering from wall (reversing, no V2X)
        #   YELLOW = V2X overtaking mode is active (always shown when V2X on)
        #   CYAN   = normal forward driving
        if self._dbg_pred_colors:
            if self.USE_OBSTACLE_AVOIDANCE and self._v2x_avoidance_active:
                self._pred_marker_color = YELLOW
            elif self._recovery_state == "RECOVERING":
                self._pred_marker_color = RED
            else:
                self._pred_marker_color = CYAN


        # === CORRIDOR CHECK (must come BEFORE get_control()) ===
        import time
        t_corridor_start = time.time()
        # Compute corridor width first. If too narrow, issue a direct ackermann stop
        # and return immediately — never let the MPC solver run in this case, because
        # the solver will produce high acceleration that the rest of the loop can't override.
        min_corridor_width = 0.15 # Just 15cm of wiggle room for the center of the kart
        curr_width_val = self._compute_corridor_width_at_ego()
        corridor_is_too_narrow = bool(curr_width_val < min_corridor_width) if self.USE_OBSTACLE_AVOIDANCE else False

        if corridor_is_too_narrow:
            if self._recovery_state != "NARROW":
                self._recovery_state = "NARROW"
                self.get_logger().warn(
                    f"Corridor width ({curr_width_val:.2f}m) < min width ({min_corridor_width:.2f}m)!"
                    " Switching to NARROW state — braking in REVERSE gear.",
                    throttle_duration_sec=1.0
                )
                gear_msg = GearCommand()
                gear_msg.stamp = self.get_clock().now().to_msg()
                gear_msg.command = GearCommand.REVERSE
                self._gear_pub.publish(gear_msg)

            if self._dbg_pred_colors:
                self._pred_marker_color = RED

            # Compute braking acceleration: full brake if moving forward
            if abs(v) > 0.1:
                stop_u = np.array([abs(self._recovery_reverse_vel), self._last_u[1]])
                stop_acc = self._mpc_cfg.a_max
            else:
                stop_u = np.array([0.0, self._last_u[1]])
                stop_acc = 0.0

            stop_acc = self._last_acc + (stop_acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
            self._last_acc = stop_acc
            self._last_u[0] = stop_u[0]
            self._car.drive([v, self._last_u[1]])
            self._publish_control_command(now, stop_u, stop_acc, False)
            self._sim_logger.log(self._car, stop_u, t)
            self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, stop_u, self._mpc, self._car)
            return
        else:
            if self._recovery_state == "NARROW":
                self.get_logger().info("Corridor is wide enough. Exiting NARROW state, resuming NORMAL driving.")
                self._recovery_state = "NORMAL"
                # Reset MPC state to warm start correctly
                self._mpc.previous_steering = self._last_u[1]
                self._mpc.infeasibility_counter = 0
                self._mpc.current_prediction = None
                
                gear_msg = GearCommand()
                gear_msg.stamp = self.get_clock().now().to_msg()
                gear_msg.command = GearCommand.DRIVE
                self._gear_pub.publish(gear_msg)

        # === CALCULATE V_MAX TARGET ===
        if self._mpc_cfg.use_csv_velocity:
            current_wp = self._reference_path.waypoints[self._mpc.model.wp_id]
            # Both are in m/s
            target_v_max_mps = min(current_wp.v_ref, self._mpc_cfg.v_max)
        else:
            if self._ref_vel_configulator is not None:
                ref_vel_mps = kmh_to_m_per_sec(self._ref_vel_configulator.get_ref_vel(self._mpc.model.wp_id))
                target_v_max_mps = min(ref_vel_mps, self._mpc_cfg.v_max)
                v_ref: List[float] = [target_v_max_mps] * len(self._reference_path.waypoints)
                self._reference_path.set_v_ref(v_ref)
            else:
                target_v_max_mps = self._mpc_cfg.v_max

        # === TAILING & LAUNCH MODE LOGIC ===
        is_overtaking_active = False
        is_actively_tailing = False
        self._is_launch_mode_active = False
        self._is_active_hard_stop = False
        in_no_overtake_zone = False
        self._current_launch_N = None
        self._corridor_bias = []
        lateral_offset = 0.0
        nearest_vid = None
        
        ego_wp_id = self._mpc.model.wp_id if self._mpc is not None else 0
        for zone in getattr(self, '_no_overtake_zones', []):
            if len(zone) == 2 and zone[0] <= ego_wp_id <= zone[1]:
                in_no_overtake_zone = True
                break

        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False) and getattr(self, '_recovery_state', "NORMAL") == "NORMAL":
            nearest_vid = getattr(self, '_nearest_opponent_vid', None)
            min_ds = getattr(self, '_nearest_opponent_ds', float('inf'))
            
            if nearest_vid is not None:
                opp_vx, opp_vy = self._v2x_tracker.velocity(nearest_vid)
                opp_speed_mps = np.hypot(opp_vx, opp_vy)
                opp_speed_kmph = opp_speed_mps * 3.6
                
                # Determine if opponent is FAST or SLOW based on threshold
                is_fast_oppo = (opp_speed_kmph >= self._v2x_fast_threshold)
                
                # Select the appropriate config dictionary
                if is_fast_oppo:
                    # [FAST OPPONENT]
                    oppo_cfg = self._fast_opponent_cfg
                    active_tailing_speed_gain = float(getattr(oppo_cfg, 'tailing_speed_gain', 0.5)) if oppo_cfg else 0.5
                    active_tailing_speed_min_delta = float(getattr(oppo_cfg, 'tailing_speed_min_delta', 2.0)) if oppo_cfg else 2.0
                    active_tailing_speed_safe_delta = float(getattr(oppo_cfg, 'tailing_speed_safe_delta', 0.2)) if oppo_cfg else 0.2
                    active_tailing_lateral_offset = float(getattr(oppo_cfg, 'tailing_lateral_offset', 0.7)) if oppo_cfg else 0.7
                    active_launch_safe_width = float(getattr(oppo_cfg, 'launch_safe_width', 0.2)) if oppo_cfg else 0.2
                    active_tailing_distance = float(getattr(oppo_cfg, 'tailing_distance_m', self._v2x_tailing_distance)) if oppo_cfg else self._v2x_tailing_distance
                    raw_gain_cfg = getattr(oppo_cfg, 'min_overtaking_steer_gain', 1.2) if oppo_cfg else 1.2
                    current_v_kmh = abs(v) * 3.6
                    active_min_overtaking_steer_gain = self._interpolate_gain(raw_gain_cfg, current_v_kmh, default_gain=1.2)
                    self._min_overtaking_steer_gain = active_min_overtaking_steer_gain
                else:
                    # [SLOW OPPONENT]
                    oppo_cfg = getattr(self, '_slow_opponent_cfg', None)
                    active_tailing_speed_gain = float(getattr(oppo_cfg, 'tailing_speed_gain', 0.5)) if oppo_cfg else 0.5
                    active_tailing_speed_min_delta = float(getattr(oppo_cfg, 'tailing_speed_min_delta', 2.0)) if oppo_cfg else 2.0
                    active_tailing_speed_safe_delta = float(getattr(oppo_cfg, 'tailing_speed_safe_delta', 0.2)) if oppo_cfg else 0.2
                    active_tailing_lateral_offset = float(getattr(oppo_cfg, 'tailing_lateral_offset', 0.7)) if oppo_cfg else 0.7
                    active_launch_safe_width = float(getattr(oppo_cfg, 'launch_safe_width', 0.2)) if oppo_cfg else 0.2
                    active_tailing_distance = float(getattr(oppo_cfg, 'tailing_distance_m', self._v2x_tailing_distance)) if oppo_cfg else self._v2x_tailing_distance
                    raw_gain_cfg = getattr(oppo_cfg, 'min_overtaking_steer_gain', [[10.0, 2.5], [20.0, 2.0], [35.0, 1.6]]) if oppo_cfg else [[10.0, 2.5], [20.0, 2.0], [35.0, 1.6]]
                    current_v_kmh = abs(v) * 3.6
                    active_min_overtaking_steer_gain = self._interpolate_gain(raw_gain_cfg, current_v_kmh, default_gain=2.5)
                    self._min_overtaking_steer_gain = active_min_overtaking_steer_gain

                active_launch_n = int(getattr(oppo_cfg, 'launch_n', 20)) if oppo_cfg else 20
                active_overtake_N = int(getattr(oppo_cfg, 'overtake_N', self._default_mpc_N)) if oppo_cfg else self._default_mpc_N
                self._current_overtake_N = active_overtake_N
                active_no_overtake_zones = getattr(oppo_cfg, 'no_overtake_zones', []) if oppo_cfg else []
                self._corridor_bias = getattr(oppo_cfg, 'corridor_bias', []) if oppo_cfg else []
                
                # LOGGING EXTRACTED PARAMETERS TO CONFIRM WHICH CONFIG IS BEING USED
                if oppo_cfg is not None:
                    oppo_type_str = "FAST" if is_fast_oppo else "SLOW"
                    self.get_logger().info(
                        f"[{oppo_type_str} OPPO PARAMS] Applied.",
                        throttle_duration_sec=1.0
                    )
                
                # Check if ego is in the active PREP zone (aggressive tailing, no launching)
                in_prep_zone = False
                for zone in active_no_overtake_zones:
                    if len(zone) >= 2 and zone[0] <= ego_wp_id <= zone[1]:
                        in_prep_zone = True
                        break
                        
                # Check if ego is in the STRICT NO-OVERTAKE zone (safe tailing, no launching)
                in_strict_zone = False
                for zone in self._strict_no_overtake_zones:
                    if len(zone) >= 2 and zone[0] <= ego_wp_id <= zone[1]:
                        in_strict_zone = True
                        break

                self._current_launch_N = active_launch_n
                
                # Convert the N-steps into a spatial launch distance
                dt = 1.0 / self._mpc_cfg.control_rate # 0.025s
                dynamic_launch_dist = self._current_launch_N * dt * target_v_max_mps
                
                # Add hysteresis so it doesn't flutter on/off at the boundary
                hysteresis_dist = dynamic_launch_dist + 4.0 if getattr(self, '_prev_launch_mode_active', False) else dynamic_launch_dist
                launch_distance_reached = (min_ds <= hysteresis_dist)
                
                # Dynamic tailing distance: Follow the config strictly.
                dynamic_tailing_dist = active_tailing_distance
                
                if in_strict_zone:
                    # STRICT Tailing for safe distance keeping
                    speed_delta = max(self._strict_tailing_min_delta, min_ds * self._strict_tailing_speed_gain) # m/s
                else:
                    # PREP / AGGRESSIVE Tailing for gaining up velocity for a pass (config based)
                    speed_delta = max(active_tailing_speed_min_delta, min_ds * active_tailing_speed_gain) # m/s
                
                tail_speed_mps = opp_speed_mps + speed_delta

                if not in_prep_zone and not in_strict_zone:
                    # Evaluate the true physical bottleneck of the corridor
                    # instead of relying on the MPC bounds which might be suppressed during tailing
                    bottleneck_width = self._reference_path.check_corridor_width(
                        ego_wp_id, 
                        int(self._mpc_cfg.N), 
                        self._car.safety_margin, 
                        self._dynamic_obstacles, 
                        min_segment_length=0.1
                    )
                    space_is_wide_enough = bottleneck_width >= active_launch_safe_width
                    
                    if launch_distance_reached and space_is_wide_enough:
                        # Launch mode activate!
                        self._is_launch_mode_active = True
                        is_overtaking_active = True
                        # target_v_max_mps remains self._mpc_cfg.v_max
                        self.get_logger().info(f"[LAUNCH MODE] {'FAST' if is_fast_oppo else 'SLOW'} Oppo! Launch distance reached ({min_ds:.1f}m)! Full speed & min steer gain: {active_min_overtaking_steer_gain:.1f}", throttle_duration_sec=1.0)
                    else:
                        # Tailing outside all zones
                        safe_wait_distance = dynamic_launch_dist + 16.0
                        
                        if min_ds <= safe_wait_distance and not space_is_wide_enough:
                            self.get_logger().info(f"[WAITING FOR GAP] Gap is too narrow ({bottleneck_width:.2f}m < {active_launch_safe_width:.2f}m). Pre-positioning and using safe delta.", throttle_duration_sec=1.0)
                            
                            # Complete physical stop if we are too close and the gap is blocked
                            stop_dist = 5.0
                            if min_ds < stop_dist:
                                tail_speed_mps = 0.0
                                self._is_active_hard_stop = False
                                self.get_logger().warn(f"Gap is still blocked and we are only {min_ds:.1f}m away! Forcing complete stop!", throttle_duration_sec=0.5)
                            else:
                                # Approach the obstacle smoothly, target to stop right at the stop_dist
                                approach_delta = (min_ds - stop_dist) * active_tailing_speed_gain
                                tail_speed_mps = opp_speed_mps + max(active_tailing_speed_safe_delta, approach_delta)
                            
                        if min_ds < dynamic_tailing_dist:
                            target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                            is_actively_tailing = True
                            self.get_logger().info(f"[TAILING] {'FAST' if is_fast_oppo else 'SLOW'} Oppo '{nearest_vid}' is {min_ds:.1f}m ahead. Speed: {opp_speed_kmph:.1f} km/h. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)
                else: # Inside either prep zone or strict zone
                    # Inside the zone, ONLY trigger emergency pass if opponent is practically crashed.
                    oppo_crashed_inside = (opp_speed_kmph < 3.5)
                    
                    if oppo_crashed_inside:
                        # Overtake (Emergency Pass) immediately!
                        self._is_launch_mode_active = True
                        is_overtaking_active = True
                        self.get_logger().info(f"[LAUNCH MODE] Zone Override (wp={ego_wp_id})! Opponent crashed/slow. Executing pass immediately. Min steer gain: {active_min_overtaking_steer_gain:.1f}", throttle_duration_sec=1.0)
                    elif min_ds < dynamic_tailing_dist:
                        # Tailing inside a zone
                        target_v_max_mps = min(target_v_max_mps, tail_speed_mps)
                        is_actively_tailing = True
                        zone_type = "STRICT ZONE" if in_strict_zone else "PREP ZONE"
                        self.get_logger().info(f"[TAILING] {zone_type} (wp={ego_wp_id}). Tailing '{nearest_vid}' at {min_ds:.1f}m. Target v_max: {target_v_max_mps*3.6:.1f} km/h.", throttle_duration_sec=1.0)

                # ===============================================
                # APPLY TAILING LATERAL OFFSET
                # ===============================================
                if is_actively_tailing and not self._is_launch_mode_active:
                    chosen_side = getattr(self._reference_path, 'obstacle_choices', {}).get(nearest_vid, "NONE")
                    if chosen_side in ["LEFT", "LEFT (smooth)", "LEFT (only valid)"]:
                        lateral_offset = active_tailing_lateral_offset
                    elif chosen_side in ["RIGHT", "RIGHT (smooth)", "RIGHT (only valid)"]:
                        lateral_offset = -active_tailing_lateral_offset

                    if self._loop % (self._mpc_cfg.control_rate // 2) == 0:
                        print(f"[TAILING OFFSET] side: {chosen_side} | offset: {lateral_offset:.2f}m", flush=True)

        if getattr(self, '_prev_launch_mode_active', False) and not self._is_launch_mode_active:
            self.get_logger().info("[LAUNCH MODE] Overtaking maneuver finished. Returning to normal steer gain.", throttle_duration_sec=2.0)
            
        if getattr(self, '_is_start_line_override', False):
            self._is_launch_mode_active = True
            is_overtaking_active = True
            self.get_logger().info("[START LINE OVERRIDE] Slamming the gas! Breaking out of the pack.", throttle_duration_sec=1.0)

        self._prev_launch_mode_active = self._is_launch_mode_active

        # === MPC HORIZON HOT-SWAP LOGIC ===
        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            target_N = getattr(self, '_current_overtake_N', self._default_mpc_N)
            
            # "Remove the mask" when standing still (v ~ 0) so the MPC can see far enough to wake up!
            if abs(v) < 0.5:
                target_N = getattr(self, '_mpc_cfg_overtake_N', 18)
                
            if self._mpc_cfg is not None and int(self._mpc_cfg.N) != target_N:
                self.get_logger().info(f"[V2X ACTIVE] Initiating MPC horizon hot-swap to N={target_N} for upcoming maneuver!")
            self._set_mpc_horizon(target_N)
        else:
            self._set_mpc_horizon(self._default_mpc_N)

        # === SMOOTH RAMP DOWN OF V_MAX ===
        # If we instantly drop v_max in the MPC constraints, the solver assumes the kart is ALREADY slow
        # and plans tight steering. But the physical kart is still flying at high speed, causing a crash!
        # We must limit the drop rate of v_max to what the physical kart can actually brake.
        dt = 1.0 / self._mpc_cfg.control_rate
        # max_decel_step_mps is the maximum drop in speed (m/s) per control cycle.
        # Use a conservative limit (0.8x of physical a_min) so the PID can keep up.
        max_decel_step_mps = abs(self._mpc_cfg.a_min) * dt * 0.8
        
        # Start the ramp down from the kart's actual physical speed (plus a small margin) 
        # instead of the previously commanded v_max, so the MPC doesn't assume we are braking
        # from a speed we never actually reached.
        current_v_max_mps = self._mpc.input_constraints['umax'][0]
        actual_speed_mps = abs(v)
        effective_start_v_max = min(current_v_max_mps, actual_speed_mps + 1.0)
        
        if getattr(self, '_is_launch_mode_active', False):
            new_v_max_mps = self._mpc_cfg.v_max
        elif target_v_max_mps < effective_start_v_max:
            new_v_max_mps = max(target_v_max_mps, effective_start_v_max - max_decel_step_mps)
        else:
            new_v_max_mps = target_v_max_mps
            
        if self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False):
            # Print detailed tailing speed logs
            if self._loop % (self._mpc_cfg.control_rate // 2) == 0:
                print(f"[TAILING-SPEED] actual_v: {actual_speed_mps*3.6:.1f} | current_v_max: {current_v_max_mps*3.6:.1f} | eff_start: {effective_start_v_max*3.6:.1f} | target_v_max: {target_v_max_mps*3.6:.1f} | new_v_max: {new_v_max_mps*3.6:.1f} (km/h)", flush=True)
            
        self._mpc.update_v_max(new_v_max_mps)

        # === MPC SOLVER ===
        with self._stats.time_block("control"):
            dynamic_obs = self._dynamic_obstacles if (self.USE_OBSTACLE_AVOIDANCE and getattr(self, '_v2x_avoidance_active', False)) else []
            launch_n_arg = getattr(self, '_current_launch_N', None) if is_overtaking_active else None
            
            # (GNSS yaw hack was removed because the GNSS error naturally forced the kart to steer right 
            # and avoid the stationary opponent in Gate 2. We keep the error so it self-realigns!)
            
            # --- START ALLEY EXIT MODE (Gate 3) ---
            import math
            # Calculate spatial state manually to check if we are in the alley
            self._car.get_current_waypoint()
            temp_spatial_state = self._car.t2s(reference_state=self._car.temporal_state, reference_waypoint=self._car.current_waypoint)
            current_ey = temp_spatial_state.e_y
            current_epsi = temp_spatial_state.e_psi
            ego_wp_id = self._car.wp_id if hasattr(self._car, 'wp_id') else 0
            
            # Forcefully NO MPC if ey is larger than 7 AND we are actually at Gate 3!
            # (Gate 3 alley is around wp_id 320 to 350)
            if abs(current_ey) > 7.0 and 300 <= ego_wp_id <= 360:
                self._in_alley_exit_mode = True
                self._alley_exit_finished = False

            if not getattr(self, '_alley_exit_finished', True):
                # Once we are close to the track centerline and aligned, hand over to MPC
                if getattr(self, '_in_alley_exit_mode', False) and abs(current_ey) < 0.3 and abs(current_epsi) < 0.2:
                    self._in_alley_exit_mode = False
                    self._alley_exit_finished = True
                    self.get_logger().info("Exited alley! Handing over to MPC.")

                if getattr(self, '_in_alley_exit_mode', False):
                    ego_wp_id = self._car.wp_id
                    
                    lookahead_distance = 10.0
                    
                    # Find lookahead waypoint on the main track
                    target_wp = None
                    s_curr = self._waypoint_s[ego_wp_id]
                    for i in range(1, 40):
                        idx = (ego_wp_id + i) % len(self._waypoint_s)
                        s_next = self._waypoint_s[idx]
                        ds = (s_next - s_curr) % self._total_track_s
                        if ds >= lookahead_distance:
                            target_wp = self._car.reference_path.waypoints[idx]
                            break
                    
                    if target_wp is None:
                        target_wp = self._car.reference_path.waypoints[(ego_wp_id + 10) % len(self._waypoint_s)]
                    
                    # ---------------------------------------------------------
                    # ALLEY EXIT RULES
                    # ---------------------------------------------------------
                    # 1. Slight turn left to avoid hitting the right wall on spawn
                    # 2. Go perfectly straight down the pitlane
                    # 3. Turn right to merge onto the track
                    
                    force_straight = False
                    force_steer = None
                    target_ey = 0.0
                    max_steer = 10.0

                    if 335 <= ego_wp_id <= 340 or ego_wp_id < 1:
                        # 1. Slight turn left
                        force_steer = 0.05 # positive is left
                    elif 1 <= ego_wp_id <= 3:
                        # 2. Go perfectly straight
                        force_straight = True
                    elif 3 <= ego_wp_id <= 6:
                        # 2. Go perfectly straight
                        force_steer = -0.25
                    else:
                        # 3. Turn right to merge (pure pursuit to track center)
                        target_ey = 0.0
                    
                    if force_steer is not None:
                        delta = force_steer
                    elif force_straight:
                        delta = 0.0
                    else:
                        # Shift the target_wp perpendicular to the track's heading
                        fake_x = target_wp.x - target_ey * math.sin(target_wp.psi)
                        fake_y = target_wp.y + target_ey * math.cos(target_wp.psi)
                        
                        # Publish target as a Marker so it's visible in RViz
                        from visualization_msgs.msg import Marker
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = now.to_msg()
                        marker.ns = "pure_pursuit"
                        marker.id = 0
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = fake_x
                        marker.pose.position.y = fake_y
                        marker.pose.position.z = 0.5
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 1.5
                        marker.scale.y = 1.5
                        marker.scale.z = 1.5
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 1.0 # Purple sphere
                        marker.color.a = 1.0
                        self._pure_pursuit_pub.publish(marker)
                        
                        dx = fake_x - self._car.temporal_state.x
                        dy = fake_y - self._car.temporal_state.y
                        target_yaw = math.atan2(dy, dx)
                        alpha = target_yaw - self._car.temporal_state.psi
                        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
                        
                        L_d = math.hypot(dx, dy)
                        L_wb = self._mpc_cfg.wheel_base if hasattr(self._mpc_cfg, 'wheel_base') else 1.087
                        
                        delta = math.atan2(2 * L_wb * math.sin(alpha), L_d)
                        
                        # Allow a strong right turn to avoid the left wall
                        max_steer = self._mpc_cfg.max_steer_angle if hasattr(self._mpc_cfg, 'max_steer_angle') else 0.64
                        delta = float(np.clip(delta, -max_steer, max_steer))


                    
                    safe_v = 1.5 # m/s
                    u = np.array([safe_v, delta])
                    acc = self.KP * (safe_v - v)
                    acc = float(np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max))
                    
                    # low pass filter for smoothness just like normal MPC output
                    acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
                    u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain
                    
                    self._last_acc = acc
                    self._last_u[0] = u[0]
                    self._last_u[1] = u[1]
                    self._car.drive([v, u[1]])
                    
                    self._publish_control_command(now, u, acc, False)
                    return
            # --- END ALLEY EXIT MODE ---
            
            u, max_delta = self._mpc.get_control(dynamic_obstacles=dynamic_obs, launch_N=launch_n_arg, lateral_offset=lateral_offset, nearest_vid=nearest_vid)

        # Dynamically retrieve vehicle width from bicycle_model (from config.yaml or parameter updates)
        # (prevents kart from trying to squeeze through gaps narrower than the configured vehicle width)
        # NOTE: If we reach this line, corridor_is_too_narrow was False (we already returned above if True).
        corridor_is_too_narrow = False  # satisfies recovery state-machine reference below

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
            
        # === MANUAL GAS OVERRIDE (START LINE) ===
        # The MPC might still plan to brake (u[0] ~ 0) if the gap is tight or due to tailing constraints.
        # We manually slam the gas to 10 m/s (36 km/h) to physically force the kart forward.
        if getattr(self, '_is_start_line_override', False):
            u[0] = max(u[0], 10.0)


        # Wall Recovery State Machine
        is_recovering = False
        if self._recovery_enabled and self._enable_control and not corridor_is_too_narrow:
            now_sec = t

            if self._recovery_state == "RECOVERING":
                if (now_sec - self._recovering_start_time) < self._recovery_reverse_time_sec:
                    is_recovering = True
                    # AWSIM: REVERSE gear + positive acc/speed moves the kart backward
                    u[0] = abs(self._recovery_reverse_vel) / 3.6
                    
                    e_y = self._car.spatial_state.e_y
                    e_psi = self._car.spatial_state.e_psi
                    
                    # When reversing, steering right (negative) moves the REAR to the right,
                    # and points the NOSE to the left.
                    # If e_y > 0 (kart is left of path), we want the rear to go right. Steer right (negative).
                    # If e_psi > 0 (nose is pointing left), we want nose to go right. Steer left (positive) in reverse.
                    steer_cmd = 0.0 * e_y + 0.5 * e_psi
                    u[1] = np.clip(steer_cmd, -0.314, 0.314)
                    
                    self.get_logger().info(f"[RECOVERY DEBUG] Reverse e_y: {e_y:.3f}, e_psi: {e_psi:.3f}, steer: {u[1]:.3f}", throttle_duration_sec=0.5)
                    
                    # Override steer gain to fixed recovery gain during recovery
                    self._mpc_cfg.steering_tire_angle_gain_var = self._recovery_steer_gain
                else:
                    self.get_logger().info("Wall recovery completed! Switching to DRIVE, aligning forward...")
                    self._recovery_state = "COOLDOWN"
                    self._cooldown_start_time = now_sec
                    self._stuck_start_time = None
                    self._recovering_start_time = None
                    self._last_u = np.array([0.0, 0.0])
                    self._last_acc = 0.0
                    self._car.update_reference_path(self._car.reference_path)
                    self._mpc_cfg.steering_tire_angle_gain_var = self._current_post_recovery_steer_gain
                    # Shift back to DRIVE gear so forward motion resumes
                    gear_msg = GearCommand()
                    gear_msg.stamp = self.get_clock().now().to_msg()
                    gear_msg.command = GearCommand.DRIVE
                    self._gear_pub.publish(gear_msg)

            elif self._recovery_state == "COOLDOWN":
                self._mpc_cfg.steering_tire_angle_gain_var = self._post_recovery_steer_gain
                
                # Unconditionally override MPC during COOLDOWN to manually align kart
                # Crawl forward at 3.6 km/h
                u[0] = 1.4  
                
                e_y = self._car.spatial_state.e_y
                e_psi = self._car.spatial_state.e_psi
                
                # Proportional control for path following in forward gear
                # If e_y > 0 (left of path), steer right (negative)
                # If e_psi > 0 (pointing left of path), steer right (negative)
                steer_cmd = -0.2 * e_y - 0.5 * e_psi
                u[1] = np.clip(steer_cmd, -0.314, 0.314)
                
                self.get_logger().info(f"[RECOVERY DEBUG] Forward Align e_y: {e_y:.3f}, e_psi: {e_psi:.3f}, steer: {u[1]:.3f}", throttle_duration_sec=0.5)
                
                # Stuck detection during alignment:
                if abs(v) < self._recovery_stuck_vel_threshold:
                    if self._stuck_start_time is None:
                        self._stuck_start_time = now_sec
                        self.get_logger().warn("Stuck timer started in COOLDOWN...", throttle_duration_sec=1.0)
                    elif (now_sec - self._stuck_start_time) >= self._recovery_stuck_time_sec:
                        self.get_logger().warn("Stuck detected during COOLDOWN! Switching back to RECOVERING...")
                        self._recovery_state = "RECOVERING"
                        self._recovering_start_time = now_sec
                        self._stuck_start_time = None
                        gear_msg = GearCommand()
                        gear_msg.stamp = self.get_clock().now().to_msg()
                        gear_msg.command = GearCommand.REVERSE
                        self._gear_pub.publish(gear_msg)
                else:
                    self._stuck_start_time = None

                # Exit condition: close to centerline and heading aligned
                is_aligned = abs(e_y) < 1.8 and abs(e_psi) < 0.15
                is_timeout = self._cooldown_start_time is not None and (now_sec - self._cooldown_start_time) >= 10.0
                
                if self._recovery_state == "COOLDOWN" and (is_aligned or is_timeout):
                    self._recovery_state = "NORMAL"
                    self._cooldown_start_time = None
                    self._set_Q0(self._default_Q0)
                    if is_aligned:
                        self.get_logger().info("Recovery alignment done. Kart is on centerline. Back to NORMAL.")
                    else:
                        self.get_logger().warn("Recovery alignment timed out. Forcing NORMAL.")

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
                    # Suppress when V2X is active (intentional stop for overtaking),
                    # corridor is too narrow (kart waiting for gap), or control is disabled (manual).
                    # When MPC crashes (infeasibility), it uses leftover predictions for up to 1.25s (50 frames).
                    # During this playback, the planned velocity might dip, which would reset the stuck timer!
                    # If the solver is failing (infeasibility_counter > 0), we must keep the stuck timer running.
                    # We use target_v_max_mps (intent) rather than u[0] (output) because if stuck against a wall, MPC might brake (u[0]=0).
                    is_commanding_forward = (target_v_max_mps > 1.0 or self._mpc.infeasibility_counter > 0)

                    # Dynamic stuck time: if we are trying to go fast and are stopped, trigger quickly (0.4s).
                    # If we intentionally stopped (tailing a stopped opponent), give it a longer grace period (1.5s)
                    # before deciding we are trapped and need to reverse out.
                    active_stuck_time_sec = self._recovery_stuck_time_sec if is_commanding_forward else 3.0

                    if self._has_moved and abs(v) < self._recovery_stuck_vel_threshold \
                            and not corridor_is_too_narrow and self._enable_control \
                            and not is_actively_tailing \
                            and (now_sec - self._node_start_time) >= 8.0:
                        if self._stuck_start_time is None:
                            self._stuck_start_time = now_sec
                            self.get_logger().warn("Stuck timer started...", throttle_duration_sec=1.0)
                        elif (now_sec - self._stuck_start_time) >= active_stuck_time_sec:
                            self.get_logger().warn(f"Stuck detected at wp {curr_wp_id}! Switching to REVERSE gear, recovering for {self._recovery_reverse_time_sec}s...")
                            self._recovery_state = "RECOVERING"
                            self._set_Q0(self._recovery_cooldown_Q0)
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
        if corridor_is_too_narrow:
            bug_acc_enabled = False
            if abs(v) > 0.05:
                acc = self._mpc_cfg.a_min
            else:
                acc = 0.0
            self._pred_marker_color = RED
            self.get_logger().warn(
                f"Corridor width ({curr_width_val:.2f}m) < vehicle width ({min_corridor_width:.2f}m)! Decelerating to full stop.",
                throttle_duration_sec=1.0
            )
        elif getattr(self, '_is_active_hard_stop', False):
            bug_acc_enabled = False
            acc = self._mpc_cfg.a_min
            self._pred_marker_color = RED
            self.get_logger().warn("ACTIVE HARD STOP OVERRIDE: Braking at a_min while maintaining MPC steering!", throttle_duration_sec=0.5)
        elif is_recovering:
            bug_acc_enabled = False
            # AWSIM: REVERSE gear + positive acc moves backward
            acc = self.KP * (u[0] - abs(v))
            acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)
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
        if hasattr(self._mpc, 'current_prediction_raw') and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
            self._publish_mpc_pred_marker(self._mpc.current_prediction_raw) # type: ignore
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
