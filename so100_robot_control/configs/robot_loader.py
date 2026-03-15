"""
YAML-based robot configuration loader.

Drop a YAML file (see robots/so100.example.yaml for the schema) and pass its
path to ``load_robot_config`` to get a fully constructed SO100Config without
editing any Python source.

Typical use
-----------
>>> from so100_robot_control.configs.robot_loader import load_robot_config
>>> config = load_robot_config("robots/so100.yaml")
>>> robot_interface = RobotInterface(config=config)
"""
from __future__ import annotations

import os
from pathlib import Path

import yaml

from so100_robot_control.configs.configs import (
    SO100Config,
    _DEFAULT_URDF,
    _DEFAULT_CALIBRATION,
)


def load_robot_config(yaml_path: str) -> SO100Config:
    """
    Build an SO100Config from a YAML file.

    Required YAML keys
    ------------------
    port : str  — serial port, e.g. /dev/tty.usbmodem58FA0919081

    Optional YAML keys
    ------------------
    name : str             — display name (informational only)
    robot_id : str         — calibration filename stem (default: so100)
    urdf : str             — path to URDF (absolute or relative to the YAML file)
    calibration_dir : str  — path to calibration directory
    calibrate : bool       — run interactive calibration on connect (default: false)
    max_relative_target : float | null
    use_degrees : bool     — default true

    Returns
    -------
    SO100Config
    """
    yaml_path = Path(yaml_path).expanduser().resolve()
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f) or {}

    port = cfg.get("port")
    if not port:
        raise ValueError(f"'port' is required in {yaml_path}")

    # Resolve URDF path relative to the YAML file if not absolute
    urdf = cfg.get("urdf", _DEFAULT_URDF)
    if urdf and not os.path.isabs(urdf):
        urdf = str(yaml_path.parent / urdf)

    # Resolve calibration dir similarly
    cal_dir = cfg.get("calibration_dir", _DEFAULT_CALIBRATION)
    if cal_dir and not os.path.isabs(cal_dir):
        cal_dir = str(yaml_path.parent / cal_dir)

    return SO100Config(
        urdf_path=urdf,
        calibration_dir=cal_dir,
        robot_id=cfg.get("robot_id", "so100"),
        port=port,
        calibrate=bool(cfg.get("calibrate", False)),
        max_relative_target=cfg.get("max_relative_target"),
        use_degrees=bool(cfg.get("use_degrees", True)),
    )
