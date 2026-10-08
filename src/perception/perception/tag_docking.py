"""Pure logic behind `tag_dock_server`: docking against a measured AprilTag.

The process (`/dock_robot`, a nav2_msgs/DockRobot action answered by
tag_dock_server):

  1. Navigate (Nav2) to the requested dock's pre-dock pose: straight out from
     the dock, `PREDOCK_TAG_DISTANCE_M` (0.75 m, the middle of the box) from
     its wall, facing it. Repeat until the robot is inside the acceptable
     pre-dock box (`Tolerance`, `predock_error`): a distance range from the
     wall, a lateral tolerance either side of the approach line and a heading
     tolerance.
  2. In the box, check that the dock bay is free (`occupied_cell_points`,
     `points_in_dock_bay`, `bay_occupied` on the local costmap, which lidar and
     camera both fill). If something is in it, wait `dock_occupied_wait_sec`
     and check again, without limit.
  3. In the box and still, switch the AprilTag detector on (`image_gate`) and
     measure that dock's tag. Once a few consecutive measurements agree
     (`MeasurementWindow`), recompute the dock pose from the tag and write it
     back to docks.yaml and docking.yaml (`capture_from_docking_pose`).
  4. Turn round (Nav2 NavigateToPose onto the staging pose: the pre-dock
     position, facing away) and hand the measured dock pose to
     opennav_docking's docking_server, which reverses into the dock
     (`dock_backwards: true`) at the limits in docking.yaml's `controller:`
     block. The robot leaves nose-first, so its camera, which sees what the
     lidar plane passes over, faces the room on undocking.

There is no hand-written manoeuvre: Nav2 brings the robot into the box and
turns it round, and the approach is Nav2's docking controller.

If the tag is never measured, the stored pose is used, with a warning. No limit
is placed on how far a measurement may move a dock (a project decision); only
the requested dock's tag id is used.

Frames: measurements are taken and averaged in `odom` (continuous while the
robot drives) and converted to `map` for goals. `capture_from_tag_transform`,
the maths `capture_dock_pose` also uses, is frame-agnostic, so a live
measurement and a stored capture agree by construction.

`plan_alignment` and the wall-gap helpers are no longer used by the server;
they stay, with their tests, for a future manoeuvre.

Nothing here touches rclpy or tf2; see `test/test_tag_docking.py`.
"""

import math
from dataclasses import dataclass

import yaml

from perception.dock_capture import (DOCKING_CHASSIS_OFFSET_M,
                                     PREDOCK_TAG_DISTANCE_M, Pose2D,
                                     normalize_angle)

# Pre-dock distance from the docked pose (itself `DOCKING_CHASSIS_OFFSET_M` out
# from the wall), putting the pre-dock pose `PREDOCK_TAG_DISTANCE_M` from the
# wall: the middle of the box.
PREDOCK_STANDOFF_M = PREDOCK_TAG_DISTANCE_M - DOCKING_CHASSIS_OFFSET_M


@dataclass(frozen=True)
class DockEntry:
    """One dock from docking.yaml.

    Attributes:
        letter: Dock letter.
        tag_id: AprilTag id.
        pose: opennav_docking's dock pose in `map`: where base_footprint must
            end up, nose-forward into the dock.
    """

    letter: str
    tag_id: int
    pose: Pose2D


def load_dock_database(yaml_text: str) -> dict:
    """Parses docking.yaml's `docks:` list.

    Args:
        yaml_text: The contents of docking.yaml.

    Returns:
        Dock letter to `DockEntry`.

    Raises:
        ValueError: If a dock's `frame` is not `map`, since another frame
            would be silently misplaced.
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
    """Re-expresses `child` (in `parent`'s frame) in `parent`'s parent frame."""
    cos_t, sin_t = math.cos(parent.theta), math.sin(parent.theta)
    return Pose2D(
        x=parent.x + cos_t * child.x - sin_t * child.y,
        y=parent.y + sin_t * child.x + cos_t * child.y,
        theta=normalize_angle(parent.theta + child.theta))


def inverse(pose: Pose2D) -> Pose2D:
    """Returns the inverse planar transform of `pose`."""
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    return Pose2D(
        x=-(cos_t * pose.x + sin_t * pose.y),
        y=-(-sin_t * pose.x + cos_t * pose.y),
        theta=normalize_angle(-pose.theta))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Returns the planar heading of a quaternion."""
    return normalize_angle(
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def correction(stored: Pose2D, measured: Pose2D):
    """Returns `(dx, dy, dtheta)` from `stored` to `measured`, in one frame."""
    return (measured.x - stored.x, measured.y - stored.y,
            normalize_angle(measured.theta - stored.theta))


def mean_pose(poses) -> Pose2D:
    """Returns the mean of planar poses, averaging headings on the circle.

    Raises:
        ValueError: If `poses` is empty.
    """
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
    """Returns the pre-dock pose: `standoff_m` straight back, same heading.

    Identical to opennav_docking's staging pose for `staging_x_offset` of
    minus `standoff_m` and `staging_yaw_offset: 0.0`.

    Args:
        dock_pose: The dock pose.
        standoff_m: Distance back from the dock pose in metres.
    """
    return Pose2D(
        x=dock_pose.x - standoff_m * math.cos(dock_pose.theta),
        y=dock_pose.y - standoff_m * math.sin(dock_pose.theta),
        theta=dock_pose.theta)


@dataclass(frozen=True)
class PredockError:
    """Robot pose relative to the pre-dock pose, in the pre-dock's own axes.

    Attributes:
        along: Positive if the robot is closer to the dock than the pre-dock.
        lateral: Positive if the robot is left of the approach line.
        heading: Positive if the robot is turned left of the approach heading.
    """

    along: float
    lateral: float
    heading: float


def predock_error(robot: Pose2D, predock: Pose2D) -> PredockError:
    """Returns `robot`'s pose relative to `predock`."""
    relative = compose(inverse(predock), robot)
    return PredockError(along=relative.x, lateral=relative.y,
                        heading=relative.theta)


@dataclass(frozen=True)
class Tolerance:
    """How far from the pre-dock pose the robot may be and still dock.

    Attributes:
        lateral_m: Accepted lateral error either side, metres.
        along_m: Half-width of the accepted along-track window, metres.
        heading_rad: Accepted heading error either side, radians.
        along_centre_m: Centre of the along-track window relative to the
            pre-dock pose (the window is along_centre_m +- along_m).
    """

    lateral_m: float
    along_m: float
    heading_rad: float
    along_centre_m: float = 0.0

    def accepts(self, error: PredockError) -> bool:
        """Returns True if `error` is inside the tolerance."""
        return (abs(error.lateral) <= self.lateral_m
                and abs(error.along - self.along_centre_m) <= self.along_m
                and abs(error.heading) <= self.heading_rad)

    def scaled(self, factor: float) -> 'Tolerance':
        """Returns a copy with the three widths multiplied by `factor`."""
        return Tolerance(self.lateral_m * factor, self.along_m * factor,
                         self.heading_rad * factor, self.along_centre_m)

    @classmethod
    def from_tag_distance(cls, lateral_m: float, heading_rad: float,
                          min_tag_distance_m: float, max_tag_distance_m: float,
                          predock_tag_distance_m: float) -> 'Tolerance':
        """Builds a tolerance accepting any distance between two wall distances.

        Args:
            lateral_m: Accepted lateral error either side, metres.
            heading_rad: Accepted heading error either side, radians.
            min_tag_distance_m: Closest accepted distance from the wall.
            max_tag_distance_m: Furthest accepted distance from the wall.
            predock_tag_distance_m: The pre-dock pose's own distance from the
                wall. Being closer to the wall is a positive `along` error.
        """
        return cls(lateral_m=lateral_m,
                   along_m=0.5 * (max_tag_distance_m - min_tag_distance_m),
                   heading_rad=heading_rad,
                   along_centre_m=predock_tag_distance_m
                   - 0.5 * (min_tag_distance_m + max_tag_distance_m))


def needs_replan(current_goal: Pose2D, new_goal: Pose2D,
                 distance_m: float, angle_rad: float) -> bool:
    """Returns whether a re-measured pre-dock pose should redirect Nav2.

    Small changes are ignored so detection jitter does not keep preempting the
    goal.

    Args:
        current_goal: The pose Nav2 is heading for.
        new_goal: The re-measured pose.
        distance_m: Position change that counts as different.
        angle_rad: Heading change that counts as different.
    """
    moved = math.hypot(new_goal.x - current_goal.x, new_goal.y - current_goal.y)
    turned = abs(normalize_angle(new_goal.theta - current_goal.theta))
    return moved > distance_m or turned > angle_rad


# --------------------------------------------------------------------------
# Measurement window
# --------------------------------------------------------------------------

class MeasurementWindow:
    """The last few measurements of one tag, and whether they agree.

    One AprilTag pose can be off by centimetres (motion blur, an oblique view,
    image/odometry timing). A pose is trusted only once the newest `required`
    measurements, all younger than `max_age_sec`, lie within `spread_m` and
    `spread_rad` of their mean.
    """

    def __init__(self, required: int, spread_m: float, spread_rad: float,
                 max_age_sec: float):
        """Initializes the window.

        Args:
            required: Number of agreeing measurements needed.
            spread_m: Largest allowed distance from the mean, metres.
            spread_rad: Largest allowed heading difference from the mean.
            max_age_sec: Oldest a measurement may be, seconds.
        """
        self.required = required
        self.spread_m = spread_m
        self.spread_rad = spread_rad
        self.max_age_sec = max_age_sec
        self._samples = []  # (time_sec, Pose2D), oldest first

    def clear(self) -> None:
        """Discards all measurements."""
        self._samples = []

    def add(self, pose: Pose2D, now_sec: float) -> None:
        """Adds a measurement taken at `now_sec`."""
        self._samples.append((now_sec, pose))
        del self._samples[:-self.required]

    def stable(self, now_sec: float):
        """Returns the mean of the newest `required` samples if they agree, else None."""
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


# Longest reverse move plan_alignment makes. The camera faces forward, so a
# reverse is checked by the lidar alone, and Nav2's BackUp collision check
# aborts long ones near walls (seen live: 0.78 m towards dock H).
MAX_BACKUP_M = 0.35


def plan_alignment(robot: Pose2D, target: Pose2D,
                   min_move_m: float = 0.01, min_turn_rad: float = 0.02,
                   max_backup_m: float = MAX_BACKUP_M):
    """Plans the moves taking a skid-steer robot from `robot` onto `target`.

    Turns to face the target, drives straight to it, then turns to its heading.
    A target behind the robot and at most `max_backup_m` away is reached by
    backing up, so a short move never pivots more than 90 deg (small turns near
    a wall). Further away the robot turns round and drives forward, camera
    first. Moves below `min_move_m` or `min_turn_rad` are dropped.

    Args:
        robot: Current pose.
        target: Wanted pose.
        min_move_m: Smallest drive kept, metres.
        min_turn_rad: Smallest turn kept, radians.
        max_backup_m: Longest reverse move allowed, metres.

    Returns:
        A list of `(kind, amount)`: `(SPIN, yaw_rad)` (positive is left),
        `(FORWARD, metres)` or `(BACKWARD, metres)`.
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

# Chassis front edge ahead of base_footprint: base_link.stl bbox x-max 0.12260
# plus base_link's +0.00046 offset (see the docking.yaml header).
NOSE_OFFSET_M = 0.123


def wall_gap_ahead(points_xy, nose_offset_m: float = NOSE_OFFSET_M,
                   half_width_m: float = 0.06, min_points: int = 3):
    """Returns the gap from the front edge to the wall straight ahead, or None.

    Only points in a corridor `half_width_m` either side of the centreline and
    in front of the nose count. The gap is their median x minus the nose offset,
    so a stray return or two does not move it.

    Args:
        points_xy: Lidar returns in base_footprint (x forward, y left).
        nose_offset_m: Front edge ahead of base_footprint, metres.
        half_width_m: Half-width of the corridor, metres.
        min_points: Fewest points needed; below it the result is None.
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
    """Returns the gap from the front edge to the tag's wall, from the tag pose.

    The wall lies `chassis_offset_m` beyond `docking_pose` along its heading,
    and the gap is measured along that heading.

    Args:
        robot: Current robot pose.
        docking_pose: capture_from_tag_transform's docking pose: base_footprint
            `chassis_offset_m` out from the wall, heading into it.
        chassis_offset_m: Docked base_footprint distance from the wall.
        nose_offset_m: Front edge ahead of base_footprint, metres.
    """
    wall_x = docking_pose.x + chassis_offset_m * math.cos(docking_pose.theta)
    wall_y = docking_pose.y + chassis_offset_m * math.sin(docking_pose.theta)
    along = ((wall_x - robot.x) * math.cos(docking_pose.theta)
             + (wall_y - robot.y) * math.sin(docking_pose.theta))
    return along - nose_offset_m


def predock_wall_gap(standoff_m: float = PREDOCK_STANDOFF_M,
                     chassis_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                     nose_offset_m: float = NOSE_OFFSET_M) -> float:
    """Returns the front-edge-to-wall gap at the pre-dock pose (~0.63 m)."""
    return standoff_m + chassis_offset_m - nose_offset_m


def undock_backup_distance(gap_m, predock_gap_m: float, min_move_m: float = 0.02):
    """Returns how far to back up to undock: to the pre-dock gap, or 0.

    0 when the wall is not seen (`gap_m` None) or the robot is already at least
    that far out, i.e. not docked.

    Args:
        gap_m: Current front-edge-to-wall gap in metres, or None.
        predock_gap_m: Gap at the pre-dock pose in metres.
        min_move_m: Smallest backup worth making, metres.
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
    """Counts the points inside the dock's bay.

    The bay is the rectangle in front of the back wall between `min_out_m` and
    `max_out_m` out from it, `half_width_m` either side of the approach line.
    `min_out_m` keeps the wall's own returns and their noise out of the count;
    `half_width_m` keeps the flared wings out.

    Args:
        points_xy: Points in the map frame.
        docking_pose: The dock's pose in docking.yaml: the docked robot pose,
            heading into the wall, `chassis_offset_m` out from it.
        min_out_m: Nearest distance out from the wall counted, metres.
        max_out_m: Furthest distance out from the wall counted, metres.
        half_width_m: Half-width of the bay, metres.
        chassis_offset_m: Docked base_footprint distance from the wall.
    """
    cos_t, sin_t = math.cos(docking_pose.theta), math.sin(docking_pose.theta)
    count = 0
    for x, y in points_xy:
        dx, dy = x - docking_pose.x, y - docking_pose.y
        along = dx * cos_t + dy * sin_t  # Positive towards the wall.
        lateral = -dx * sin_t + dy * cos_t
        out = chassis_offset_m - along  # Distance out from the wall.
        if min_out_m <= out <= max_out_m and abs(lateral) <= half_width_m:
            count += 1
    return count


def points_near(points_xy, pose: Pose2D, radius_m: float) -> int:
    """Counts the points within `radius_m` of `pose`'s x, y.

    Applied to occupied costmap cells, it tells whether something stands on a
    pose the robot is about to be sent to (the pre-dock pose).

    Args:
        points_xy: Points in the map frame.
        pose: The pose to test around.
        radius_m: Radius in metres.
    """
    return sum(1 for x, y in points_xy
               if math.hypot(x - pose.x, y - pose.y) <= radius_m)


def occupied_cell_points(data, width: int, height: int, resolution: float,
                         origin_x: float, origin_y: float, occupied_value: int):
    """Returns the centres of the occupied cells, in the grid's frame.

    The grid is taken as axis-aligned, as the local costmap's rolling window is.
    For Nav2's costmap `occupied_value` is 100 (lethal, a marked obstacle); the
    inflated cells around an obstacle are 99 and below and unknown cells are
    -1, so neither counts.

    Args:
        data: Row-major `data` of a nav_msgs/OccupancyGrid.
        width: Grid width in cells.
        height: Grid height in cells.
        resolution: Cell size in metres.
        origin_x: X of `info.origin` in metres.
        origin_y: Y of `info.origin` in metres.
        occupied_value: Smallest cell value counted as occupied.

    Returns:
        A list of `(x, y)` cell centres.
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
    """Returns whether the bay holds something.

    Occupied when at least half the costmaps each have `min_points` or more
    occupied cells in the bay, so a stray cell or one odd update does not count.

    Args:
        counts: One `points_in_dock_bay` count per costmap received at the
            pre-dock pose.
        min_points: Occupied cells needed for one costmap to count.

    Returns:
        True or False, or None if there were no costmaps: nothing was seen,
        which is not the same as the bay being free.
    """
    counts = list(counts)
    if not counts:
        return None
    hits = sum(1 for count in counts if count >= min_points)
    return 2 * hits >= len(counts)

