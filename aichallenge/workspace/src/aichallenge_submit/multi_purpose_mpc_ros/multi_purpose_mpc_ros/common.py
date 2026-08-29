from typing import Dict, List, NamedTuple, Union

import os
import argparse
from collections import namedtuple

import rclpy

def parse_args_without_ros(argv):
    args_without_ros = rclpy.utilities.remove_ros_args(argv) # type: ignore
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config_path", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-r", "--ref_vel_path", type=str, required=False,
                        help="Path to the ref_vel.yaml file")
    parser.add_argument("-s", "--steer_gain_path", type=str, required=False,
                        help="Path to the steer_gain.yaml file")
    return parser.parse_args(args_without_ros[1:])

# 再帰的に dict を namedtuple に変換する関数
def convert_to_namedtuple(
        data: Union[Dict, List, NamedTuple, float, str, bool],
        tuple_name="Config"
        ) -> Union[Dict, List, NamedTuple, float, str, bool]:
    if isinstance(data, dict):
        fields = {key: convert_to_namedtuple(value, key) for key, value in data.items()}
        return namedtuple(tuple_name, fields.keys())(**fields)
    elif isinstance(data, list):
        return [convert_to_namedtuple(item, tuple_name) for item in data]
    else:
        return data

def file_exists(file_path: str) -> None:
    if not os.path.exists(file_path):
        raise FileNotFoundError("File not found: " + file_path)

def read_config_file(file_path: str) -> str:
    if file_path.endswith('.enc'):
        import base64
        import zlib
        with open(file_path, "rb") as f:
            content_bytes = f.read()
        return zlib.decompress(base64.b85decode(content_bytes)).decode('utf-8')
    else:
        with open(file_path, "r") as f:
            return f.read()
