"""Guards config/scan_filter.yaml against silently becoming a pass-through.

A ROS 2 params file only applies to a node whose name matches its top-level
key. If the launch files rename the laser_filters node (name='scan_filter')
but the YAML stays keyed on the executable's default name
(scan_to_scan_filter_chain), the node starts with NO filterN parameters, runs
an empty chain and republishes /scan unchanged - no error, no warning.
"""

import ast
import math
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
YAML_PATH = os.path.join(PKG, 'config', 'scan_filter.yaml')
LAUNCH_FILES = ['mapping.launch.py', 'localize.launch.py']

# Real mounting measured on the robot with
# `ros2 run tf2_ros tf2_echo base_link base_lidar_link` (2026-10-03):
# translation [0.040, 0.000, 0.040], yaw +90 deg.
LIDAR_X = 0.040
LIDAR_YAW = math.pi / 2


def _load():
    with open(YAML_PATH) as f:
        return yaml.safe_load(f)


def _scan_filter_node_names(launch_file):
    """Names given to laser_filters scan_to_scan_filter_chain Nodes."""
    with open(os.path.join(PKG, 'launch', launch_file)) as f:
        tree = ast.parse(f.read())
    names = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        exe = kw.get('executable')
        if isinstance(exe, ast.Constant) and \
                exe.value == 'scan_to_scan_filter_chain':
            names.append(kw['name'].value if 'name' in kw
                         else 'scan_to_scan_filter_chain')
    return names


def _box():
    params = _load()
    (key,) = params.keys()
    return params[key]['ros__parameters']['filter1']['params']


def test_yaml_key_matches_node_name_in_every_launch_file():
    keys = set(_load().keys())
    for lf in LAUNCH_FILES:
        names = _scan_filter_node_names(lf)
        assert names, f'{lf} starts no scan_to_scan_filter_chain node'
        for name in names:
            assert name in keys or '/**' in keys, (
                f'{lf} names the filter node {name!r} but scan_filter.yaml '
                f'is keyed {sorted(keys)}: the filter chain would be empty')


def test_box_matches_spec_starts_at_lidar_20cm_back_10cm_each_side():
    box = _box()
    assert box['box_frame'] == 'base_link'
    assert math.isclose(box['max_x'], LIDAR_X)
    assert math.isclose(box['min_x'], LIDAR_X - 0.20)
    assert math.isclose(box['min_y'], -0.10)
    assert math.isclose(box['max_y'], 0.10)
    assert box['invert'] is False


def test_box_contains_measured_antenna_returns():
    # Antenna returns seen in a live /scan on the robot (2026-10-03):
    # (lidar-frame angle deg, range m). Converted to base_link via the TF.
    box = _box()
    hits = [(48, 0.134), (49, 0.135), (50, 0.136),
            (128, 0.134), (129, 0.132), (130, 0.133), (132, 0.136)]
    for deg, r in hits:
        a = math.radians(deg) + LIDAR_YAW
        x = LIDAR_X + r * math.cos(a)
        y = r * math.sin(a)
        assert box['min_x'] <= x <= box['max_x'], (deg, r, x)
        assert box['min_y'] <= y <= box['max_y'], (deg, r, y)
