#!/usr/bin/env python3

from typing import Dict, List, Tuple, Optional
import os
import shutil
from datetime import datetime
import copy
import yaml
from collections import OrderedDict

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Multi_Purpose_MPC
from multi_purpose_mpc_ros.tools.reference_path_generator import ReferencePathGenerator

RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class SteerGainConfigurator:
    MIN_GAIN = 0.8
    MAX_GAIN = 1.8

    def __init__(
        self,
        node: Node,
        ref_path_config_path: str,
        steer_gain_config_path: str,
    ) -> None:
        self._node = node
        self._steer_gain_config_path = steer_gain_config_path
        self._reference_path = ReferencePathGenerator.get_reference_path(ref_path_config_path)
        self._steer_gain_cfg = self._load_config(steer_gain_config_path)
        
        # Load raw ref_vel config to track parameter updates
        ref_vel_path = os.path.join(os.path.dirname(self._steer_gain_config_path), "ref_vel.yaml")
        try:
            with open(ref_vel_path, "r") as f:
                self._ref_vel_cfg = yaml.safe_load(f).get("ref_vel_configulator", {})
        except Exception:
            self._ref_vel_cfg = {}

        self._ref_vel_map = self._load_ref_vel_map()
        self._label_step = 5

        self._sorted_wp_id_gain_map: OrderedDict[int, float] = OrderedDict()
        self._setup_publisher()
        self._register_params()

        self._ref_path_marker_array = MarkerArray()
        self._section_marker_array = MarkerArray()
        self._update_marker()
        self._timer = self._node.create_timer(2.0, self._timer_callback)

    @classmethod
    def _load_config(cls, config_path: str) -> Dict:
        with open(config_path, "r") as f:
            cfg: Dict = yaml.safe_load(f)
        return cfg["steer_gain_configurator"]

    def _load_ref_vel_map(self) -> Dict[int, float]:
        ref_vel_path = os.path.join(os.path.dirname(self._steer_gain_config_path), "ref_vel.yaml")
        if not os.path.exists(ref_vel_path):
            return {}
        try:
            with open(ref_vel_path, "r") as f:
                cfg = yaml.safe_load(f)
            ref_cfg = cfg.get("ref_vel_configulator", {})
            res = {}
            for k, v in ref_cfg.items():
                if isinstance(v, dict) and "wp_id" in v and "ref_vel" in v:
                    res[int(v["wp_id"])] = float(v["ref_vel"])
            return res
        except Exception:
            return {}

    def get_ref_vel(self, wp_id: int) -> float:
        if not self._ref_vel_map:
            return 30.0
        sorted_keys = sorted(self._ref_vel_map.keys())
        for i in range(len(sorted_keys)):
            start = sorted_keys[i]
            end = sorted_keys[(i + 1) % len(sorted_keys)]
            if start <= end:
                if start <= wp_id < end:
                    return self._ref_vel_map[start]
            else:
                if wp_id >= start or wp_id < end:
                    return self._ref_vel_map[start]
        return list(self._ref_vel_map.values())[0]

    def _register_params(self):
        def declare_parameters():
            for section_name, info in self._steer_gain_cfg.items():
                for key, value in info.items():
                    if key != "wp_id" and key != "gain":
                        raise ValueError(f"Invalid key in steer_gain config: {key}")
                    self._node.declare_parameter(f"steer_gain/{section_name}/{key}", value)

            self._node.declare_parameter("steer_gain/save", False)
            self._node.declare_parameter("steer_gain/label_step", 5)

        def param_cb(parameters):
            update_needed = False
            for param in parameters:
                if param.name.startswith("steer_gain/"):
                    if param.name == "steer_gain/save":
                        # backup current config file
                        current_datetime = datetime.now().strftime('%Y%m%d_%H%M%S')
                        backup_filename = f"{self._steer_gain_config_path}.{current_datetime}"
                        if os.path.exists(self._steer_gain_config_path):
                            shutil.copy2(self._steer_gain_config_path, backup_filename)

                        cfg_for_save = { "steer_gain_configurator": self._steer_gain_cfg }
                        # save current config
                        with open(self._steer_gain_config_path, 'w', encoding='utf-8') as file:
                            yaml.dump(cfg_for_save, file, allow_unicode=True, default_flow_style=False)
                    elif param.name == "steer_gain/label_step":
                        self._label_step = max(1, int(param.value))
                        self._node.get_logger().info(f"Updated steer_gain label_step to {self._label_step}")
                        update_needed = True
                    else:
                        parts = param.name.split("/")
                        if len(parts) == 3:
                            _, section_name, param_name = parts
                            if section_name in self._steer_gain_cfg:
                                self._steer_gain_cfg[section_name][param_name] = param.value
                                update_needed = True
                elif param.name.startswith("ref_vel/"):
                    parts = param.name.split("/")
                    if len(parts) == 3 and self._ref_vel_cfg is not None:
                        _, section_name, param_name = parts
                        if section_name in self._ref_vel_cfg:
                            self._ref_vel_cfg[section_name][param_name] = param.value
                            # Rebuild _ref_vel_map from memory
                            res = {}
                            for k, v in self._ref_vel_cfg.items():
                                if isinstance(v, dict) and "wp_id" in v and "ref_vel" in v:
                                    res[int(v["wp_id"])] = float(v["ref_vel"])
                            self._ref_vel_map = res
                            update_needed = True

            if update_needed:
                self._update_marker()
            return SetParametersResult(successful=True)

        declare_parameters()
        self._node.add_on_set_parameters_callback(param_cb)

    def _setup_publisher(self) -> None:
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._steer_gain_marker_pub = self._node.create_publisher(
            MarkerArray, "/steer_gain_marker", latching_qos
        )
        self._section_marker_pub = self._node.create_publisher(
            MarkerArray, "/steer_gain_section_marker", latching_qos
        )

    def _timer_callback(self) -> None:
        self._steer_gain_marker_pub.publish(self._ref_path_marker_array)
        self._section_marker_pub.publish(self._section_marker_array)

    def _update_marker(self):
        class Section:
            def __init__(self, name, gain):
                self.name = name
                self.gain = gain

        sorted_dict = sorted(
            (
                (int(v["wp_id"]), Section(k, float(v["gain"])))
                for k, v in self._steer_gain_cfg.items()
            ),
            key=lambda item: int(item[0]),
        )
        wp_id_section_map: OrderedDict[int, Section] = OrderedDict(sorted_dict)

        self._sorted_wp_id_gain_map = OrderedDict(
            sorted(
                (
                    (int(v["wp_id"]), float(v["gain"]))
                    for k, v in self._steer_gain_cfg.items()
                ),
                key=lambda item: int(item[0]),
            )
        )

        def add_ref_path_marker(markers: MarkerArray) -> None:
            line_base = Marker()
            line_base.header.frame_id = "map"
            line_base.ns = "steer_gain_path"
            line_base.type = Marker.LINE_STRIP
            line_base.action = Marker.ADD
            line_base.pose.position.z = 0.1
            line_base.scale.x = 0.15

            ref_path = self._reference_path
            for i in range(len(ref_path.waypoints) - 1):
                gain = self.get_steer_gain(i, interpolate=True)
                line = copy.deepcopy(line_base)
                line.id = i
                line.color = self.create_gain_heat_color(gain)
                start = Point()
                start.x = ref_path.waypoints[i].x
                start.y = ref_path.waypoints[i].y
                start.z = 0.1
                end = Point()
                end.x = ref_path.waypoints[i + 1].x
                end.y = ref_path.waypoints[i + 1].y
                end.z = 0.1
                line.points.append(start)
                line.points.append(end)
                markers.markers.append(line)

        def add_section_markers(markers: MarkerArray) -> None:
            spheres = Marker()
            spheres.header.frame_id = "map"
            spheres.ns = "steer_section_point"
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            radius = 0.3
            spheres.scale = Vector3(x=radius, y=radius, z=radius)
            spheres.color = GREEN

            text_base = Marker()
            text_base.header.frame_id = "map"
            text_base.type = Marker.TEXT_VIEW_FACING
            text_base.action = Marker.ADD
            text_base.pose.position.z = 0.0
            text_base.scale.z = 0.4

            total_wps = len(self._reference_path.waypoints)
            step = max(1, self._label_step)

            for wp_id in range(0, total_wps, step):
                p = Point()
                p.x = self._reference_path.waypoints[wp_id].x
                p.y = self._reference_path.waypoints[wp_id].y
                p.z = 2.5
                spheres.points.append(p)

                gain = self.get_steer_gain(wp_id, interpolate=True)
                ref_vel = self.get_ref_vel(wp_id)

                text = copy.deepcopy(text_base)
                text.id = wp_id
                text.ns = f"steer_gain_wp_label"
                text.pose.position = copy.deepcopy(p)
                text.pose.position.x += 1.0
                text.pose.position.y += 1.0
                text.text = f"wp:{wp_id}\nvel:{ref_vel:.1f} km/h\ngain:{gain:.2f}"
                text.color = self.create_gain_heat_color(gain)
                markers.markers.append(text)

            markers.markers.append(spheres)

        self._ref_path_marker_array = MarkerArray()
        add_ref_path_marker(self._ref_path_marker_array)
        self._section_marker_array = MarkerArray()
        add_section_markers(self._section_marker_array)

    def get_steer_gain(self, current_wp_id: int, interpolate: bool = True) -> float:
        if not self._sorted_wp_id_gain_map:
            return 1.0

        sorted_keys = list(self._sorted_wp_id_gain_map.keys())
        num_keys = len(sorted_keys)
        total_wps = len(self._reference_path.waypoints) if self._reference_path.waypoints else 356

        if not interpolate:
            for i in range(num_keys):
                start = sorted_keys[i]
                end = sorted_keys[(i + 1) % num_keys]
                target_gain = self._sorted_wp_id_gain_map[start]
                if start <= end:
                    if start <= current_wp_id < end:
                        return target_gain
                else:
                    if current_wp_id >= start or current_wp_id < end:
                        return target_gain
            return list(self._sorted_wp_id_gain_map.values())[0]

        # Linear Interpolation
        for i in range(num_keys):
            start_wp = sorted_keys[i]
            end_wp = sorted_keys[(i + 1) % num_keys]
            start_gain = self._sorted_wp_id_gain_map[start_wp]
            end_gain = self._sorted_wp_id_gain_map[end_wp]

            if start_wp < end_wp:
                if start_wp <= current_wp_id <= end_wp:
                    dist = end_wp - start_wp
                    if dist == 0:
                        return start_gain
                    ratio = (current_wp_id - start_wp) / dist
                    return start_gain + ratio * (end_gain - start_gain)
            else:
                # Wrap-around across start/finish line
                if current_wp_id >= start_wp or current_wp_id <= end_wp:
                    total_dist = (total_wps - start_wp) + end_wp
                    if total_dist <= 0:
                        return start_gain
                    if current_wp_id >= start_wp:
                        curr_dist = current_wp_id - start_wp
                    else:
                        curr_dist = (total_wps - start_wp) + current_wp_id
                    ratio = min(1.0, max(0.0, curr_dist / total_dist))
                    return start_gain + ratio * (end_gain - start_gain)

        return list(self._sorted_wp_id_gain_map.values())[0]

    @classmethod
    def create_gain_heat_color(cls, gain: float, alpha: float = 1.0) -> ColorRGBA:
        norm = max(0.0, min((gain - cls.MIN_GAIN) / (cls.MAX_GAIN - cls.MIN_GAIN), 1.0))
        r = norm
        g = 1.0 - norm
        b = 0.2
        return ColorRGBA(r=r, g=g, b=b, a=alpha)
