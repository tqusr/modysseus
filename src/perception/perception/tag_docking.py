"""Pure logic behind `tag_dock_server`: docking against a measured AprilTag.

THE PROCESS (`/dock_robot`, answered by tag_dock_server; a nav2_msgs/DockRobot action):

  1. Navigate (Nav2) to the PRE-DOCK pose of the requested dock: straight out
     from the dock, `PREDOCK_TAG_DISTANCE_M` (0.75 m, the middle of the box)
     from its wall, facing it. Repeat until the robot is inside the acceptable
     pre-dock BOX (`Tolerance`, `predock_error`): within a distance range of the
     wall, a lateral tolerance either side of the approach line and a heading
     tolerance.
  2. Standing in the box, check the dock bay is free (`occupied_cell_points`,
     `points_in_dock_bay`, `bay_occupied` on the local costmap, which the lidar
     AND the camera fill). If another robot or an obstacle is in it, wait
     `dock_occupied_wait_sec` and check again, with no limit on the number of
     checks.
  3. Standing in the box, still, switch the AprilTag detector on (`image_gate`;
     idle the rest of the time, being ~60% of a CPU core otherwise) and measure
     THAT dock's tag. Once a few consecutive measurements agree
     (`MeasurementWindow`), the dock pose is recomputed from the tag and written
     back to docks.yaml and docking.yaml (`capture_from_docking_pose`).
  4. Back in. The robot turns round (Nav2 NavigateToPose onto the STAGING pose:
     the pre-dock position, facing away from the dock) and the measured dock pose
     is handed to opennav_docking's docking_server, which REVERSES it into the
     dock (`dock_backwards: true`), at the speed limits in docking.yaml's
     `controller:` block (so the controller plans with them, rather than being
     slowed from outside). The robot leaves nose-first, so its camera, which
     sees what the lidar's plane passes over, faces the room when it undocks.

There is no hand-written manoeuvre: the robot is brought into the box and turned
round by Nav2, and the approach from there is Nav2's docking controller.

If the tag is never measured, the stored pose is used, with a warning in the
log. No limit is applied to how far a measurement may move a dock (an explicit
project decision); only the requested dock's tag id is ever used.

Frames: measurements are taken and averaged in `odom` (continuous while the
robot drives) and converted to `map` for navigation and docking goals.
`capture_from_tag_transform` -- the same maths `capture_dock_pose` uses to
write docking.yaml -- is frame-agnostic, so a live measurement and a stored
capture agree by construction.

`plan_alignment` and the wall-gap helpers below are no longer used by the
server; they stay (with their tests) as tools for a future manoeuvre.

Nothing here touches rclpy or tf2; see `test/test_tag_docking.py`.
"""

import math
from dataclasses import dataclass

import yaml

from perception.dock_capture import (DOCKING_CHASSIS_OFFSET_M,
                                     PREDOCK_TAG_DISTANCE_M, Pose2D,
                                     normalize_angle)

#: Pre-dock distance from the DOCKING pose (the robot's docked pose, which is
#: itself `DOCKING_CHASSIS_OFFSET_M` out from the wall), so that the pre-dock
#: pose lands `PREDOCK_TAG_DISTANCE_M` from the wall: the middle of the box.
PREDOCK_STANDOFF_M = PREDOCK_TAG_DISTANCE_M - DOCKING_CHASSIS_OFFSET_M


@dataclass(frozen=True)
class DockEntry:
    """One dock from docking.yaml: its letter, tag id and stored robot pose.

    `pose` is opennav_docking's dock pose in `map`, i.e. where the robot's
    base_footprint must end up, nose-forward into the dock.
    """

    letter: str
    tag_id: int
    pose: Pose2D


def load_dock_database(yaml_text: str) -> dict:
    """Parse docking.yaml's `docks:` list into {letter: DockEntry}.

    Docks whose `frame` is not `map` are rejected: every pose in this repo is a
    map pose, and silently treating another frame as map would misplace it.
    """
    params = yaml.safe_load(yaml_text)['docking_server']['ros__parameters']
    database = {}
    for letter in params['docks']:
        entry = params[letter]
        if entry.get('frame', 'map') != 'map':
            raise ValueError(
                f"dock {letter}: frame {entry['frame']!r} is not 'map'")
        x, y, theta = (float(value) for value in entry['pose'])
        database[letter] = DockEntry(
            letter=letter, tag_id=int(entry['id']),
            pose=Pose2D(x=x, y=y, theta=normalize_angle(theta)))
    return database


# --------------------------------------------------------------------------
# Planar geometry
# --------------------------------------------------------------------------

def compose(parent: Pose2D, child: Pose2D) -> Pose2D:
    """`child` (expressed in parent's frame) re-expressed in parent's parent."""
    cos_t, sin_t = math.cos(parent.theta), math.sin(parent.theta)
    return Pose2D(
        x=parent.x + cos_t * child.x - sin_t * child.y,
        y=parent.y + sin_t * child.x + cos_t * child.y,
        theta=normalize_angle(parent.theta + child.theta))


def inverse(pose: Pose2D) -> Pose2D:
    """The inverse planar transform of `pose`."""
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    return Pose2D(
        x=-(cos_t * pose.x + sin_t * pose.y),
        y=-(-sin_t * pose.x + cos_t * pose.y),
        theta=normalize_angle(-pose.theta))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Planar heading of a quaternion."""
    return normalize_angle(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def correction(stored: Pose2D, measured: Pose2D):
    """(dx, dy, dtheta) from the stored pose to the measured one, same frame."""
    return (measured.x - stored.x, measured.y - stored.y,
            normalize_angle(measured.theta - stored.theta))


def mean_pose(poses) -> Pose2D:
    """Average of planar poses; headings averaged on the circle."""
    poses = list(poses)
    if not poses:
        raise ValueError('mean of no poses')
    n = len(poses)
    return Pose2D(
        x=sum(p.x for p in poses) / n,
        y=sum(p.y for p in poses) / n,
        theta=normalize_angle(math.atan2(sum(math.sin(p.theta) for p in poses),
                                         sum(math.cos(p.theta) for p in poses))))


# --------------------------------------------------------------------------
# Pre-dock pose and tolerance
# --------------------------------------------------------------------------

def predock_pose_for(dock_pose: Pose2D,
                     standoff_m: float = PREDOCK_STANDOFF_M) -> Pose2D:
    """Pre-dock pose: `standoff_m` straight back from the dock pose, same heading.

    The robot then only has to drive forward to dock. Identical to
    opennav_docking's staging pose for docking.yaml's `staging_x_offset` set to
    minus `standoff_m` and `staging_yaw_offset: 0.0`.
    """
    return Pose2D(
        x=dock_pose.x - standoff_m * math.cos(dock_pose.theta),
        y=dock_pose.y - standoff_m * math.sin(dock_pose.theta),
        theta=dock_pose.theta)


@dataclass(frozen=True)
class PredockError:
    """Robot pose relative to the pre-dock pose, in the pre-dock's own axes.

    along    + = robot is further forward (closer to the dock) than pre-dock
    lateral  + = robot is to the left of the approach line
    heading  + = robot is turned left of the approach heading
    """

    along: float
    lateral: float
    heading: float


def predock_error(robot: Pose2D, predock: Pose2D) -> PredockError:
    relative = compose(inverse(predock), robot)
    return PredockError(along=relative.x, lateral=relative.y,
                        heading=relative.theta)


@dataclass(frozen=True)
class Tolerance:
    """How far from the pre-dock pose the robot may be and still dock."""

    lateral_m: float
    along_m: float
    heading_rad: float
    #: Centre of the accepted along-track window, relative to the pre-dock
    #: pose (the window is along_centre_m +- along_m).
    along_centre_m: float = 0.0

    def accepts(self, error: PredockError) -> bool:
        return (abs(error.lateral) <= self.lateral_m
                and abs(error.along - self.along_centre_m) <= self.along_m
                and abs(error.heading) <= self.heading_rad)

    def scaled(self, factor: float) -> 'Tolerance':
        return Tolerance(self.lateral_m * factor, self.along_m * factor,
                         self.heading_rad * factor, self.along_centre_m)

    @classmethod
    def from_tag_distance(cls, lateral_m: float, heading_rad: float,
                          min_tag_distance_m: float, max_tag_distance_m: float,
                          predock_tag_distance_m: float) -> 'Tolerance':
        """Accept any robot position between two distances from the tag's wall.

        `predock_tag_distance_m` is the pre-dock pose's own distance from the
        wall. Being closer to the wall is a positive `along` error.
        """
        return cls(lateral_m=lateral_m,
                   along_m=0.5 * (max_tag_distance_m - min_tag_distance_m),
                   heading_rad=heading_rad,
                   along_centre_m=predock_tag_distance_m
                   - 0.5 * (min_tag_distance_m + max_tag_distance_m))


def needs_replan(current_goal: Pose2D, new_goal: Pose2D,
                 distance_m: float, angle_rad: float) -> bool:
    """Whether a re-measured pre-dock pose differs enough to redirect Nav2.

    Small changes are ignored so detection jitter does not keep preempting
    the navigation goal.
    """
    moved = math.hypot(new_goal.x - current_goal.x, new_goal.y - current_goal.y)
    turned = abs(normalize_angle(new_goal.theta - current_goal.theta))
    return moved > distance_m or turned > angle_rad


# --------------------------------------------------------------------------
# Measurement window
# --------------------------------------------------------------------------

class MeasurementWindow:
    """The last few measurements of one tag, and whether they agree.

    A single AprilTag pose can be off by centimetres (motion blur while
    driving, an oblique view, timing between image and odometry). A pose is
    only trusted once the newest `required` measurements, all younger than
    `max_age_sec`, lie within `spread_m` / `spread_rad` of their mean.
    """

    def __init__(self, required: int, spread_m: float, spread_rad: float,
                 max_age_sec: float):
        self.required = required
        self.spread_m = spread_m
        self.spread_rad = spread_rad
        self.max_age_sec = max_age_sec
        self._samples = []        # (time_sec, Pose2D), oldest first

    def clear(self) -> None:
        self._samples = []

    def add(self, pose: Pose2D, now_sec: float) -> None:
        self._samples.append((now_sec, pose))
        del self._samples[:-self.required]

    def stable(self, now_sec: float):
        """Mean of the newest `required` samples if they agree, else None."""
        recent = [pose for stamp, pose in self._samples
                  if now_sec - stamp <= self.max_age_sec]
        if len(recent) < self.required:
            return None
        mean = mean_pose(recent)
        for pose in recent:
            if math.hypot(pose.x - mean.x, pose.y - mean.y) > self.spread_m:
                return None
            if abs(normalize_angle(pose.theta - mean.theta)) > self.spread_rad:
                return None
        return mean


# --------------------------------------------------------------------------
# Alignment: turn - drive - turn
# --------------------------------------------------------------------------

SPIN = 'spin'
FORWARD = 'forward'
BACKWARD = 'backward'


#: Longest move plan_alignment makes in reverse. The camera faces forward, so
#: a reverse is checked by the lidar alone and Nav2's BackUp collision check
#: aborts long ones near walls (seen live: a 0.78 m back-up towards dock H).
MAX_BACKUP_M = 0.35


def plan_alignment(robot: Pose2D, target: Pose2D,
                   min_move_m: float = 0.01, min_turn_rad: float = 0.02,
                   max_backup_m: float = MAX_BACKUP_M):
    """Moves taking a skid-steer robot from `robot` exactly onto `target`.

    Returns a list of (kind, amount): (SPIN, relative yaw in rad, + = left),
    (FORWARD, metres) or (BACKWARD, metres). Turn to face the target, drive
    straight to it, turn to its heading. When the target is behind the robot
    and at most `max_backup_m` away it backs up instead of turning around, so
    the robot never pivots more than 90 deg for a short move -- near a wall
    that keeps the turns small. Further away it turns around and drives
    forward, with the camera facing the way it goes. Moves below
    `min_move_m` / `min_turn_rad` are dropped.
    """
    moves = []
    heading = robot.theta
    distance = math.hypot(target.x - robot.x, target.y - robot.y)
    if distance >= min_move_m:
        bearing = math.atan2(target.y - robot.y, target.x - robot.x)
        turn = normalize_angle(bearing - heading)
        kind = FORWARD
        if abs(turn) > math.pi / 2 and distance <= max_backup_m:
            turn = normalize_angle(turn - math.pi)
            kind = BACKWARD
        if abs(turn) >= min_turn_rad:
            moves.append((SPIN, turn))
            heading = normalize_angle(heading + turn)
        moves.append((kind, distance))
    final_turn = normalize_angle(target.theta - heading)
    if abs(final_turn) >= min_turn_rad:
        moves.append((SPIN, final_turn))
    return moves


# --------------------------------------------------------------------------
# Final approach: distance to the wall
# --------------------------------------------------------------------------

#: How far the chassis front edge is ahead of base_footprint: base_link.stl
#: bbox x-max 0.12260 + base_link's own +0.00046 offset (docking.yaml header).
NOSE_OFFSET_M = 0.123


def wall_gap_ahead(points_xy, nose_offset_m: float = NOSE_OFFSET_M,
                   half_width_m: float = 0.06, min_points: int = 3):
    """Gap from the robot's front edge to the wall straight ahead, or None.

    `points_xy` are lidar returns in base_footprint (x forward, y left). Only
    points in a corridor `half_width_m` either side of the centreline and in
    front of the nose count; the gap is their median x minus the nose offset,
    so a stray return or two does not move it.
    """
    ahead = sorted(x for x, y in points_xy
                   if abs(y) <= half_width_m and x > nose_offset_m - 0.02)
    if len(ahead) < min_points:
        return None
    middle = len(ahead) // 2
    median = (ahead[middle] if len(ahead) % 2
              else 0.5 * (ahead[middle - 1] + ahead[middle]))
    return median - nose_offset_m


def wall_gap_from_tag(robot: Pose2D, docking_pose: Pose2D,
                      chassis_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                      nose_offset_m: float = NOSE_OFFSET_M) -> float:
    """Gap from the front edge to the tag's wall, from the measured tag pose.

    `docking_pose` is capture_from_tag_transform's docking pose: base_footprint
    `chassis_offset_m` out from the wall, heading into it. The wall is that
    far further along the heading; the gap is measured along the heading.
    """
    wall_x = docking_pose.x + chassis_offset_m * math.cos(docking_pose.theta)
    wall_y = docking_pose.y + chassis_offset_m * math.sin(docking_pose.theta)
    along = ((wall_x - robot.x) * math.cos(docking_pose.theta)
             + (wall_y - robot.y) * math.sin(docking_pose.theta))
    return along - nose_offset_m


def predock_wall_gap(standoff_m: float = PREDOCK_STANDOFF_M,
                     chassis_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                     nose_offset_m: float = NOSE_OFFSET_M) -> float:
    """Front-edge-to-wall gap when standing at the pre-dock pose (~0.63 m)."""
    return standoff_m + chassis_offset_m - nose_offset_m


def undock_backup_distance(gap_m, predock_gap_m: float, min_move_m: float = 0.02):
    """How far to back up to undock: back to the pre-dock gap, or 0.

    0 when the wall is not seen (gap None) or the robot is already at least
    that far out -- i.e. not docked, nothing to undock from.
    """
    if gap_m is None:
        return 0.0
    distance = predock_gap_m - gap_m
    return distance if distance >= min_move_m else 0.0


# --------------------------------------------------------------------------
# Is the dock bay free?
# --------------------------------------------------------------------------

def points_in_dock_bay(points_xy, docking_pose: Pose2D, min_out_m: float,
                       max_out_m: float, half_width_m: float,
                       chassis_offset_m: float = DOCKING_CHASSIS_OFFSET_M) -> int:
    """How many of `points_xy` (map frame) lie inside the dock's bay.

    The bay is the rectangle in front of the dock's back wall between
    `min_out_m` and `max_out_m` out from the wall, `half_width_m` either side of
    the approach line. `docking_pose` is the dock's pose in docking.yaml: the
    robot's docked pose, heading into the wall, `chassis_offset_m` out from it.
    `min_out_m` keeps the wall's own returns (and the lidar's noise on them) out
    of the count; `half_width_m` keeps the flared wings out of it.
    """
    cos_t, sin_t = math.cos(docking_pose.theta), math.sin(docking_pose.theta)
    count = 0
    for x, y in points_xy:
        dx, dy = x - docking_pose.x, y - docking_pose.y
        along = dx * cos_t + dy * sin_t          # + = towards the wall
        lateral = -dx * sin_t + dy * cos_t
        out = chassis_offset_m - along           # distance out from the wall
        if min_out_m <= out <= max_out_m and abs(lateral) <= half_width_m:
            count += 1
    return count


def occupied_cell_points(data, width: int, height: int, resolution: float,
                         origin_x: float, origin_y: float, occupied_value: int):
    """Centres (x, y), in the grid's frame, of the cells marked occupied.

    `data` is a nav_msgs/OccupancyGrid's row-major `data`; `origin_x/y` its
    `info.origin` position. The grid is taken as axis-aligned, which the local
    costmap's rolling window is. A cell is occupied when its value is at least
    `occupied_value`. For Nav2's costmap that is 100 (a LETHAL cell, a marked
    obstacle): the cells around an obstacle, within the robot's radius of it,
    are 99 and below, and unknown cells are -1, so neither counts.
    """
    points = []
    for index, value in enumerate(data):
        if value >= occupied_value:
            row, column = divmod(index, width)
            if row >= height:
                break
            points.append((origin_x + (column + 0.5) * resolution,
                           origin_y + (row + 0.5) * resolution))
    return points


def bay_occupied(counts, min_points: int):
    """Whether the bay holds something, from per-costmap counts of occupied cells.

    `counts` has one entry per costmap received while standing at the pre-dock
    pose (see `points_in_dock_bay`). Occupied when at least half of them each
    have `min_points` or more occupied cells inside the bay; a stray cell or a
    single odd update does not count. None when there were no costmaps at all:
    nothing was seen, which is not the same as the bay being free.
    """
    counts = list(counts)
    if not counts:
        return None
    hits = sum(1 for count in counts if count >= min_points)
    return 2 * hits >= len(counts)

