"""The docking sequence's pure parts: is the dock bay free, saving a measured
dock pose, and that the config files agree about where the pre-dock pose is.

No rclpy, no ROS graph, no robot. WHAT THESE CANNOT DO: say whether the lidar
sees a real robot in a real dock bay (the bay's size comes from the simulated
dock model), or whether Nav2's docking controller completes the approach at the
configured speed. Those are checks on the robot.
"""

import math
import os

import pytest
import yaml

from perception.dock_capture import (DOCKING_CHASSIS_OFFSET_M,
                                     PREDOCK_TAG_DISTANCE_M, Pose2D,
                                     capture_from_docking_pose,
                                     docking_server_pose_for,
                                     format_docking_entry, staging_pose_for)
from perception.tag_docking import (PREDOCK_STANDOFF_M, Tolerance, bay_occupied,
                                    occupied_cell_points, points_in_dock_bay,
                                    predock_error, predock_pose_for)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCKING_YAML = os.path.join(PKG, 'config', 'docking.yaml')
DOCKS_YAML = os.path.join(PKG, '..', 'mission', 'config', 'docks.yaml')

BAY = dict(min_out_m=0.15, max_out_m=0.55, half_width_m=0.15)


def bay_points(docking_pose, outs, laterals):
    """Points at the given distances out from the wall / sideways, in the map."""
    cos_t, sin_t = math.cos(docking_pose.theta), math.sin(docking_pose.theta)
    points = []
    for out, lateral in zip(outs, laterals):
        along = DOCKING_CHASSIS_OFFSET_M - out
        points.append((docking_pose.x + along * cos_t - lateral * sin_t,
                       docking_pose.y + along * sin_t + lateral * cos_t))
    return points


@pytest.fixture(params=[0.0, 1.0, math.pi, -2.3])
def docking_pose(request):
    return Pose2D(1.7, -0.4, request.param)


# --- the bay ---------------------------------------------------------------

def test_points_inside_the_bay_are_counted(docking_pose):
    points = bay_points(docking_pose, [0.25, 0.3, 0.4], [0.0, 0.1, -0.1])
    assert points_in_dock_bay(points, docking_pose, **BAY) == 3


def test_the_docks_own_wall_is_not_counted(docking_pose):
    wall = bay_points(docking_pose, [0.0, 0.03, 0.08], [0.0, 0.1, -0.1])
    assert points_in_dock_bay(wall, docking_pose, **BAY) == 0


def test_the_wings_beside_the_bay_are_not_counted(docking_pose):
    wings = bay_points(docking_pose, [0.3, 0.3], [0.25, -0.25])
    assert points_in_dock_bay(wings, docking_pose, **BAY) == 0


def test_points_out_in_the_room_are_not_counted(docking_pose):
    # The pre-dock pose itself is 0.75 m out; nothing at or beyond the bay edge.
    room = bay_points(docking_pose, [0.6, 0.75, 1.2], [0.0, 0.0, 0.0])
    assert points_in_dock_bay(room, docking_pose, **BAY) == 0


def test_just_inside_the_bay_edges_counts_and_just_outside_does_not(docking_pose):
    eps = 1e-4
    inside = bay_points(docking_pose, [0.15 + eps, 0.55 - eps, 0.3, 0.3],
                        [0.0, 0.0, 0.15 - eps, -0.15 + eps])
    outside = bay_points(docking_pose, [0.15 - eps, 0.55 + eps, 0.3, 0.3],
                         [0.0, 0.0, 0.15 + eps, -0.15 - eps])
    assert points_in_dock_bay(inside, docking_pose, **BAY) == 4
    assert points_in_dock_bay(outside, docking_pose, **BAY) == 0


def test_a_robot_in_the_dock_reads_as_occupied_from_the_predock_pose(docking_pose):
    """The rear face of a docked robot: ~0.23 m wide, ~0.26 m out from the wall."""
    rear = bay_points(docking_pose, [0.26] * 7,
                      [-0.11, -0.07, -0.035, 0.0, 0.035, 0.07, 0.11])
    assert points_in_dock_bay(rear, docking_pose, **BAY) == 7


def test_an_empty_dock_has_nothing_in_the_bay(docking_pose):
    assert points_in_dock_bay([], docking_pose, **BAY) == 0


# --- reading the costmap -------------------------------------------------------

def test_only_lethal_cells_become_points():
    #  width 3, height 2:  row 0 = free, inflated(99), lethal(100); row 1 = unknown, lethal, free
    data = [0, 99, 100, -1, 100, 0]
    points = occupied_cell_points(data, 3, 2, 0.05, 1.0, 2.0, 100)
    assert points == [pytest.approx((1.125, 2.025)), pytest.approx((1.075, 2.075))]


def test_the_cell_centre_is_half_a_cell_in_from_the_origin():
    assert occupied_cell_points([100], 1, 1, 0.1, -0.5, 0.5, 100) == [
        pytest.approx((-0.45, 0.55))]


def test_no_cells_no_points():
    assert occupied_cell_points([0] * 9, 3, 3, 0.05, 0.0, 0.0, 100) == []


# --- deciding from several costmap updates --------------------------------------

def test_occupied_when_half_the_scans_see_enough_points():
    assert bay_occupied([5, 4, 0, 0], 3) is True
    assert bay_occupied([6, 6, 6], 3) is True


def test_a_single_odd_scan_does_not_make_it_occupied():
    assert bay_occupied([0, 0, 4, 0, 0], 3) is False
    assert bay_occupied([0, 1, 2, 2], 3) is False     # a few strays, never enough


def test_no_scans_is_unknown_not_free():
    assert bay_occupied([], 3) is None


# --- saving a measured pose --------------------------------------------------

@pytest.mark.parametrize('theta', [0.0, 1.0, math.pi, -1.6])
def test_a_docking_pose_round_trips_through_the_capture(theta):
    docking = Pose2D(2.0, -1.0, theta)
    capture = capture_from_docking_pose(3, docking)
    again = docking_server_pose_for(capture.dock_pose)
    assert (again.x, again.y) == pytest.approx((docking.x, docking.y), abs=1e-9)
    assert math.cos(again.theta - docking.theta) == pytest.approx(1.0, abs=1e-9)
    assert capture.letter == 'D'
    assert capture.docking_pose == docking


def test_the_capture_puts_the_wall_ahead_and_the_staging_pose_in_the_middle_of_the_box():
    docking = Pose2D(0.0, 0.0, 0.0)           # heading +x, into a wall at x = 0.126
    capture = capture_from_docking_pose(0, docking)
    assert (capture.dock_pose.x, capture.dock_pose.y) == pytest.approx((0.126, 0.0))
    assert capture.staging_pose.x == pytest.approx(0.126 - PREDOCK_TAG_DISTANCE_M)
    assert capture.staging_pose == staging_pose_for(capture.dock_pose)


def test_an_unknown_tag_id_is_rejected():
    with pytest.raises(Exception):
        capture_from_docking_pose(99, Pose2D(0.0, 0.0, 0.0))


def test_the_saved_docking_line_is_the_measured_pose():
    capture = capture_from_docking_pose(1, Pose2D(1.151, -0.092, -0.0344))
    assert format_docking_entry(capture) == (
        'B: {type: "arena_dock", frame: "map", pose: [1.151, -0.092, -0.0344], '
        'id: "1"}')


# --- the config files agree --------------------------------------------------

def _docking_params():
    with open(DOCKING_YAML) as handle:
        return yaml.safe_load(handle)


def test_predock_is_the_middle_of_the_box():
    tag = _docking_params()['tag_dock_server']['ros__parameters']
    low, high = tag['predock_min_tag_distance_m'], tag['predock_max_tag_distance_m']
    assert PREDOCK_TAG_DISTANCE_M == pytest.approx(0.5 * (low + high))
    assert tag['predock_standoff_m'] == pytest.approx(PREDOCK_STANDOFF_M)
    # ...and the Tolerance built from them is centred on it.
    box = Tolerance.from_tag_distance(
        tag['predock_lateral_tolerance_m'], tag['predock_heading_tolerance_rad'],
        low, high, tag['predock_standoff_m'] + DOCKING_CHASSIS_OFFSET_M)
    assert box.along_centre_m == pytest.approx(0.0, abs=1e-9)
    assert box.accepts(predock_error(Pose2D(0, 0, 0), Pose2D(0, 0, 0)))


def test_docking_server_staging_offset_matches_the_predock():
    server = _docking_params()['docking_server']['ros__parameters']
    assert server['arena_dock']['staging_x_offset'] == pytest.approx(-PREDOCK_STANDOFF_M)


def test_docks_yaml_staging_poses_are_the_predock_poses():
    """The mission navigates to docks.yaml's staging pose; tag_dock_server
    navigates to the pre-dock pose of docking.yaml's dock pose. One place."""
    with open(DOCKS_YAML) as handle:
        docks = yaml.safe_load(handle)['docks']
    entries = _docking_params()['docking_server']['ros__parameters']
    for letter, dock in docks.items():
        x, y, theta = entries[letter]['pose']
        predock = predock_pose_for(Pose2D(x, y, theta), PREDOCK_STANDOFF_M)
        staging = dock['staging_pose']
        assert (predock.x, predock.y) == pytest.approx(
            (staging['x'], staging['y']), abs=0.003), letter
        assert math.cos(predock.theta - staging['theta']) == pytest.approx(
            1.0, abs=1e-4), letter


def test_the_dock_approach_is_slow_and_its_limits_are_consistent():
    controller = _docking_params()['docking_server']['ros__parameters']['controller']
    assert controller['v_linear_max'] == pytest.approx(0.08)
    assert controller['v_linear_min'] == pytest.approx(0.05)
    assert 0 < controller['v_linear_min'] < controller['v_linear_max']


def test_the_sequence_parameters_are_set():
    tag = _docking_params()['tag_dock_server']['ros__parameters']
    assert tag['dock_occupied_wait_sec'] == pytest.approx(10.0)
    assert tag['bay_min_out_m'] < tag['bay_max_out_m']
    assert tag['bay_min_cells'] >= 1
    assert tag['bay_occupied_value'] == 100
    assert tag['bay_costmap_topic'] == 'local_costmap/costmap'
    assert tag['save_measured_pose'] is True
    for removed in ('align_range_m', 'align_speed_mps', 'give_up_tolerance_factor',
                    'scan_topic', 'bay_min_points'):
        assert removed not in tag


def test_the_robot_backs_in_and_docking_servers_staging_pose_is_where_we_turn_to():
    """docking.yaml's staging pose (dock pose + staging_x_offset along its
    heading, heading + staging_yaw_offset) is the pre-dock position facing AWAY
    from the dock; the mission's docks.yaml staging pose is the same position
    facing the dock."""
    server = _docking_params()['docking_server']['ros__parameters']
    assert server['dock_backwards'] is True
    offset = server['arena_dock']['staging_x_offset']
    yaw_offset = server['arena_dock']['staging_yaw_offset']
    assert yaw_offset == pytest.approx(math.pi)
    with open(DOCKS_YAML) as handle:
        docks = yaml.safe_load(handle)['docks']
    for letter, dock in docks.items():
        x, y, theta = server[letter]['pose']
        sx, sy = x + offset * math.cos(theta), y + offset * math.sin(theta)
        syaw = theta + yaw_offset
        mission = dock['staging_pose']
        assert (sx, sy) == pytest.approx((mission['x'], mission['y']), abs=0.003), letter
        # facing away from the dock: opposite to the mission's staging heading
        assert math.cos(syaw - mission['theta']) == pytest.approx(-1.0, abs=1e-4), letter
        # ...and the dock pose heading is into the wall, towards the dock
        into_wall = math.atan2(dock['dock_pose']['y'] - sy, dock['dock_pose']['x'] - sx)
        assert math.cos(theta - into_wall) == pytest.approx(1.0, abs=2e-2), letter
