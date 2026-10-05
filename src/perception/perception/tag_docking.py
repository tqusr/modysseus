"""Pure logic behind `tag_dock_server`: docking against a measured AprilTag.

THE PROCESS (`/dock_robot`, answered by tag_dock_server; a nav2_msgs/DockRobot action):

  1. Navigate towards the requested dock's STORED pre-dock pose
     (docking.yaml's dock pose, `standoff_m` straight back from it).
  2. Whenever that dock's tag is in view and close enough, measure it. Once a
     few consecutive measurements agree (`MeasurementWindow`), recompute the
     dock pose and its pre-dock pose from the tag, and redirect navigation
     there if it moved noticeably (`needs_replan`).
  3. At the pre-dock pose, standing still, measure once more -- the closest,
     straight-on view, so the most accurate one -- and check the robot is
     within tolerance of the pre-dock pose (`predock_error`, `Tolerance`). If
     not, ALIGN (`plan_alignment`): turn to face the pre-dock pose, drive
     straight to it (or back up to it), turn square to the dock -- then measure
     and check again, a limited number of times.
  4. Hand the measured dock pose to opennav_docking's docking_server
     (use_dock_id: false, navigate_to_staging_pose: false), whose controller
     drives forward from the pre-dock pose into the dock and stops within its
     own `docking_threshold`.

If the tag is never measured, the stored pose is used throughout, i.e. exactly
the docking behaviour from before this feature, with a warning in the log.
No limit is applied to how far a measurement may move a dock (an explicit
project decision); only the requested dock's tag id is ever used.

WHY ALIGNMENT IS NOT LEFT TO NAV2. Nav2 declares a goal reached within its
general_goal_checker tolerance (0.15 m, 0.2 rad in
navigation/config/nav2_params.yaml), and re-sending a goal the robot is
already "at" by that measure finishes immediately without moving. Docks B, C
and D sit ~0.3 m apart on one wall, so after undocking from C the robot is
only ~0.3 m from B's pre-dock pose: Nav2 could leave it 15 cm off B's
centreline, and the "straight" drive in would come in diagonally. Nav2 is
therefore only used to get NEAR the pre-dock pose (beyond `align_range_m`);
the last part is turn-drive-turn with Nav2's own Spin / DriveOnHeading /
BackUp behaviours, which are exact over short distances for a skid-steer
robot and still go through the velocity smoother and collision monitor.

Frames: measurements are taken and averaged in `odom` (continuous while the
robot drives) and converted to `map` for navigation and docking goals.
`capture_from_tag_transform` -- the same maths `capture_dock_pose` uses to
write docking.yaml -- is frame-agnostic, so a live measurement and a stored
capture agree by construction.

Nothing here touches rclpy or tf2; see `test/test_tag_docking.py`.
"""

import math
from dataclasses import dataclass

import yaml

from perception.dock_capture import (STAGING_STANDOFF_M, Pose2D,
                                     normalize_angle)


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
                     standoff_m: float = STAGING_STANDOFF_M) -> Pose2D:
    """Pre-dock pose: `standoff_m` straight back from the dock pose, same heading.

    The robot then only has to drive forward to dock. Identical to
    opennav_docking's staging pose for docking.yaml's `staging_x_offset: -0.6`
    and `staging_yaw_offset: 0.0`.
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

    def accepts(self, error: PredockError) -> bool:
        return (abs(error.lateral) <= self.lateral_m
                and abs(error.along) <= self.along_m
                and abs(error.heading) <= self.heading_rad)

    def scaled(self, factor: float) -> 'Tolerance':
        return Tolerance(self.lateral_m * factor, self.along_m * factor,
                         self.heading_rad * factor)


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


def plan_alignment(robot: Pose2D, target: Pose2D,
                   min_move_m: float = 0.01, min_turn_rad: float = 0.02):
    """Moves taking a skid-steer robot from `robot` exactly onto `target`.

    Returns a list of (kind, amount): (SPIN, relative yaw in rad, + = left),
    (FORWARD, metres) or (BACKWARD, metres). Turn to face the target, drive
    straight to it, turn to its heading. When the target is behind the robot
    it backs up instead of turning around, so the robot never pivots more than
    90 deg to translate -- near a wall that keeps the turns small. Moves below
    `min_move_m` / `min_turn_rad` are dropped.
    """
    moves = []
    heading = robot.theta
    distance = math.hypot(target.x - robot.x, target.y - robot.y)
    if distance >= min_move_m:
        bearing = math.atan2(target.y - robot.y, target.x - robot.x)
        turn = normalize_angle(bearing - heading)
        kind = FORWARD
        if abs(turn) > math.pi / 2:
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
