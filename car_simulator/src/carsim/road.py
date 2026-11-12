"""
Road-geometry helpers shared by the scenarios and the tools.

Everything here works on a carla.Map, so it can be used both inside a
ScenarioRunner scenario and from standalone scripts.
"""

import math
import os
from dataclasses import dataclass, replace

import carla


# Maps made by CARLA 0.9.15's Digital Twin Tool (OpenDriveToMap) have raised sidewalks and, built with the stock tool,
# a road mesh that waves around the OpenDRIVE heights; actors placed at a waypoint's z end up inside or above it. Such
# maps are saved under /Game/CustomMaps/; these names count as well (an offline carla.Map is named after its file).
# CARSIM_DEFORMED=1|0 forces the answer.
DEFORMED_MAPS = ("WolfsburgHighway", "Wolfsburg2", "Wolfsburg")
# Built with digital_twin/apply_patches.py's flat roads (WolfsburgHighway from its fourth build, 2026-11-11): the mesh
# lies at the OpenDRIVE heights, only the sidewalks are raised.
FLAT_MAPS = ("WolfsburgHighway",)
DEFORMED_ENV = "CARSIM_DEFORMED"
SIDEWALK_SURFACE = 0.1524 + 0.15  # a generated sidewalk is this much over its lane's height: Lane.cpp + GenerateRoadMesh [m]


def deformation(x, y, name=""):
    """
    How far the generated road mesh lies over the OpenDRIVE height at (x, y) (CARLA coordinates) [m]; 0 on FLAT_MAPS
    (`name` is carla.Map.name). Otherwise 0.7 x
    GetZPosInDeformation of LibCarla road/Deformation.h (0.9.15), as OpenDriveToMap applies it. Checked against a car
    driving the generated map Wolfsburg2: its z minus (OpenDRIVE z + this) was +0.006 m, sd 0.019 m (735 samples).
    The driving lanes' bumps (GetBumpDeformation, at most 0.1 m) are left out: they did not improve that fit.
    """
    if name.split("/")[-1] in FLAT_MAPS:
        return 0.0
    return 0.7 * (0.6 * math.sin(0.035 * x - 0.08 * y + 1000.0) + 1.1 * math.sin(0.02 * x + 0.05 * y - 1500.0))


def deformed_map(name):
    """True for a map whose road mesh is deformed (see DEFORMED_MAPS); `name` is carla.Map.name."""
    forced = os.environ.get(DEFORMED_ENV)
    if forced in ("0", "1"):
        return forced == "1"
    return "/CustomMaps/" in name or name.split("/")[-1] in DEFORMED_MAPS


def surface_offset(cmap, location):
    """
    What to add to a z computed from the OpenDRIVE heights (a waypoint's, plus whatever lift) at `location` to be as
    high over a deformed map's mesh: deformation(x, y, map name), plus SIDEWALK_SURFACE on a Sidewalk lane.
    """
    lift = SIDEWALK_SURFACE if cmap.get_waypoint(location, project_to_road=False,
                                                 lane_type=carla.LaneType.Sidewalk) is not None else 0.0
    return deformation(location.x, location.y, getattr(cmap, "name", "")) + lift


class Crosswalk:
    """A painted crosswalk polygon from carla.Map.get_crosswalks()."""

    def __init__(self, points):
        self.points = points
        n = len(points)
        self.center = carla.Location(
            x=sum(p.x for p in points) / n,
            y=sum(p.y for p in points) / n,
            z=sum(p.z for p in points) / n)

        # The longest edge runs across the road; the walker crosses along it
        a, b = max(((points[i], points[(i + 1) % n]) for i in range(n)), key=lambda e: e[0].distance(e[1]))
        dx, dy = b.x - a.x, b.y - a.y
        norm = math.hypot(dx, dy) or 1.0
        self.axis = (dx / norm, dy / norm)

        # Extent of the polygon along the axis, relative to the center
        along = [(p.x - self.center.x) * self.axis[0] + (p.y - self.center.y) * self.axis[1] for p in points]
        self.t_min, self.t_max = min(along), max(along)


def crosswalks(cmap):
    """
    Split carla.Map.get_crosswalks() into polygons. The API returns all
    polygons as one flat list where each polygon repeats its first point
    to mark its end. The Digital Twin maps (e.g. WolfsburgHighway) give open
    outlines of 4 corners that never repeat their first point, in any order
    with closed ones: a point closes the polygon that starts 4, 8, ... points
    back (the points before it are open outlines), and the open points (left
    before a closed polygon or at the end) are split into quadrilaterals (a
    multiple of 4) or taken as one polygon (3 points or more).
    """
    def outlines(points):
        if len(points) % 4 == 0:
            return [Crosswalk(points[i:i + 4]) for i in range(0, len(points), 4)]
        return [Crosswalk(points)] if len(points) > 2 else []

    polygons, current = [], []
    for point in cmap.get_crosswalks():
        start = next((k for k in range(0, len(current) - 2, 4) if point.distance(current[k]) < 0.05), None)
        if start is None:
            current.append(point)
            continue
        polygons += outlines(current[:start]) + [Crosswalk(current[start:])]
        current = []
    return polygons + outlines(current)


class CrosswalkIndex:
    """Grid lookup so large maps (Town12 has thousands of crosswalks) stay fast."""

    def __init__(self, items, cell=25.0):
        self._cell = cell
        self._grid = {}
        for cw in items:
            self._grid.setdefault(self._key(cw.center), []).append(cw)

    def _key(self, loc):
        return int(math.floor(loc.x / self._cell)), int(math.floor(loc.y / self._cell))

    def near(self, loc, radius):
        cx, cy = self._key(loc)
        span = int(math.ceil(radius / self._cell))
        found = []
        for ix in range(cx - span, cx + span + 1):
            for iy in range(cy - span, cy + span + 1):
                found.extend(cw for cw in self._grid.get((ix, iy), ()) if cw.center.distance(loc) <= radius)
        return found


def _turn(wp, yaw):
    """Absolute heading change from `yaw` to `wp` [deg]."""
    return abs((wp.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0)


def junction_entry_ahead(wp, max_distance=500.0, step=1.0):
    """
    Follow the lane from `wp` to the last waypoint before the next junction.
    Returns (entry_waypoint, distance_travelled) or (None, distance) if no
    junction is found within `max_distance`.
    """
    travelled = 0.0
    while travelled <= max_distance:
        nxt = wp.next(step)
        if not nxt:
            return None, travelled
        if nxt[0].is_junction:
            return wp, travelled
        wp = nxt[0]
        travelled += step
    return None, travelled


def lane_point(wp, s):
    """Waypoint `s` metres from `wp` along its lane (negative = back), going straight on at junctions."""
    if s < 0:
        prev = wp.previous(-s)
        return prev[0] if prev else wp
    left = s
    while left > 1e-3:
        step = min(1.0, left)
        options = wp.next(step)
        if not options:
            break
        yaw = wp.transform.rotation.yaw
        wp = min(options, key=lambda w: _turn(w, yaw))
        left -= step
    return wp


def _walk_line_point(entry, cw, iterations=8, tolerance=0.02):
    """
    Where the lane through `entry` meets the crosswalk's walk line (through
    its center, along its axis). Returns (s, waypoint) with `s` the signed
    distance from `entry` along the lane, or None if the lane runs parallel
    or the search does not converge (e.g. the lane ends first).
    """
    nx, ny = -cw.axis[1], cw.axis[0]
    c = cw.center
    s, wp = 0.0, entry
    for _ in range(iterations):
        loc = wp.transform.location
        fwd = wp.transform.get_forward_vector()
        slope = fwd.x * nx + fwd.y * ny
        if abs(slope) < 0.5:
            return None
        ds = -((loc.x - c.x) * nx + (loc.y - c.y) * ny) / slope
        if abs(ds) < tolerance:
            return s, wp
        s += ds
        wp = lane_point(entry, s)
    return None


def crosswalk_at_junction(entry_wp, candidates, max_back=20.0, max_inside=10.0, max_dz=3.0, slack=0.5):
    """
    Pick the crosswalk the ego lane crosses at the junction entry. CARLA
    usually puts it a few metres inside the junction, sometimes just before.

    Returns (crosswalk, s, lane_wp) or None. `s` is the signed distance from
    the entry waypoint along the lane to the crosswalk's walk line (negative =
    before the junction), `lane_wp` the lane waypoint on that line. The
    crosswalk nearest to the entry wins; ones the lane does not run through
    (side-road crosswalks, other heights) are skipped.
    """
    entry_loc = entry_wp.transform.location
    fwd = entry_wp.transform.get_forward_vector()

    best = None
    for cw in candidates:
        # Matching is 2D, so reject crosswalks on another level (overpasses)
        if abs(cw.center.z - entry_loc.z) > max_dz:
            continue
        # Skip crosswalks that run along the ego road (they cross the side roads)
        if abs(cw.axis[0] * fwd.x + cw.axis[1] * fwd.y) > 0.5:
            continue
        hit = _walk_line_point(entry_wp, cw)
        if hit is None:
            continue
        s, wp = hit
        if not -max_back <= s <= max_inside:
            continue
        loc = wp.transform.location
        t = (loc.x - cw.center.x) * cw.axis[0] + (loc.y - cw.center.y) * cw.axis[1]
        if not cw.t_min - slack <= t <= cw.t_max + slack:
            continue
        if best is None or abs(s) < abs(best[1]):
            best = (cw, s, wp)
    return best


def straight_route(start_wp, distance, step=2.0):
    """
    Waypoints from `start_wp` for `distance` metres, going straight on at
    junctions (the branch with the smallest heading change).
    """
    route = []
    wp = start_wp
    travelled = 0.0
    while travelled < distance:
        options = wp.next(step)
        if not options:
            break
        yaw = wp.transform.rotation.yaw
        wp = min(options, key=lambda w: _turn(w, yaw))
        route.append(wp)
        travelled += step
    return route


def _lanes_toward(wp, side, max_steps=10):
    """
    Yield the lanes next to `wp`, going outwards on `side` ("right" or "left",
    relative to the lane's driving direction). Lanes of the opposite direction
    have their left/right flipped, which is handled here.
    """
    positive = wp.lane_id > 0
    current = wp
    for _ in range(max_steps):
        same_direction = (current.lane_id > 0) == positive
        if side == "right":
            current = current.get_right_lane() if same_direction else current.get_left_lane()
        else:
            current = current.get_left_lane() if same_direction else current.get_right_lane()
        if current is None:
            return
        yield current


def find_sidewalk(wp, side, max_steps=10):
    """Walk sideways from `wp` until a sidewalk lane is found, or return None."""
    for lane in _lanes_toward(wp, side, max_steps):
        if lane.lane_type == carla.LaneType.Sidewalk:
            return lane
    return None


def _beyond_outermost_lane(wp, side, margin):
    """Location `margin` metres past the outer edge of the outermost lane on `side` of `wp` (any lane type)."""
    lanes = list(_lanes_toward(wp, side))
    outer = lanes[-1] if lanes else wp
    right = wp.transform.get_right_vector()
    sign = 1.0 if side == "right" else -1.0
    off = sign * (outer.lane_width / 2.0 + margin)
    loc = outer.transform.location
    return carla.Location(x=loc.x + right.x * off, y=loc.y + right.y * off, z=loc.z)


def _out_of_driving_lane(cmap, loc, dx, dy, step=0.5, max_push=10.0, margin=0.3):
    """
    Move `loc` along (dx, dy) until it is at least `margin` away from the
    driving lanes (a point exactly on a lane edge can be reported either
    way). Returns None if it never gets off.
    """
    def on_lane(k):
        probe = carla.Location(x=loc.x + dx * k, y=loc.y + dy * k, z=loc.z)
        return cmap.get_waypoint(probe, project_to_road=False, lane_type=carla.LaneType.Driving) is not None

    pushed = 0.0
    while on_lane(-margin) or on_lane(0.0) or on_lane(margin):
        if pushed >= max_push:
            return None
        loc = carla.Location(x=loc.x + dx * step, y=loc.y + dy * step, z=loc.z)
        pushed += step
    return loc


def pedestrian_path(cmap, crossing_wp, crosswalk=None):
    """
    Start and end of a walk across the road at `crossing_wp`, from the
    ego's right-hand side to its left.

    With a crosswalk the walk runs along the polygon's axis, from 1 m beyond
    one end to 1 m beyond the other. Without one it runs from the right
    sidewalk to the far sidewalk; a missing sidewalk is replaced by a point 1 m
    beyond the outermost lane on that side. Each end is moved outwards until
    it is off the driving lanes.

    Returns (spawn_transform, end_location, distance), or None if an end
    cannot be moved off the driving lanes.
    """
    lane_tf = crossing_wp.transform
    right = lane_tf.get_right_vector()
    margin = 1.0
    ux, uy = right.x, right.y  # walking direction, towards the ego's right

    if crosswalk is not None:
        ux, uy = crosswalk.axis
        if ux * right.x + uy * right.y < 0:
            ux, uy = -ux, -uy
        c = crosswalk.center
        t_hi = max((p.x - c.x) * ux + (p.y - c.y) * uy for p in crosswalk.points)
        t_lo = min((p.x - c.x) * ux + (p.y - c.y) * uy for p in crosswalk.points)
        start = carla.Location(x=c.x + ux * (t_hi + margin), y=c.y + uy * (t_hi + margin), z=c.z)
        end = carla.Location(x=c.x + ux * (t_lo - margin), y=c.y + uy * (t_lo - margin), z=c.z)
    else:
        near = find_sidewalk(crossing_wp, "right")
        start = near.transform.location if near is not None else _beyond_outermost_lane(crossing_wp, "right", margin)
        far = find_sidewalk(crossing_wp, "left")
        end = far.transform.location if far is not None else _beyond_outermost_lane(crossing_wp, "left", margin)

    start = _out_of_driving_lane(cmap, start, ux, uy)
    end = _out_of_driving_lane(cmap, end, -ux, -uy)
    if start is None or end is None:
        return None

    dx, dy = end.x - start.x, end.y - start.y
    distance = math.hypot(dx, dy)
    yaw = math.degrees(math.atan2(dy, dx))
    spawn = carla.Transform(carla.Location(x=start.x, y=start.y, z=start.z + 1.0), carla.Rotation(yaw=yaw))
    return spawn, end, distance


@dataclass
class Crossing:
    """
    wp             ego-lane waypoint where the pedestrian crosses (trigger location)
    distance       signed distance from the ego start along its lane to `wp` [m]
    crosswalk      the painted crosswalk, or None
    spawn, end     pedestrian spawn transform and walk end location
    walk_distance  length of the walk [m]
    """
    wp: carla.Waypoint
    distance: float
    crosswalk: Crosswalk
    spawn: carla.Transform
    end: carla.Location
    walk_distance: float


def find_crossing(cmap, start_wp, crosswalk_index, max_distance=500.0, min_ahead=1.0):
    """
    From the ego start waypoint, find where a pedestrian should cross at the
    next junction: the crosswalk of the ego's road (before or inside the
    junction), or the junction entry itself if there is no painted crosswalk.

    The crossing is at least `min_ahead` metres ahead of the start: the
    backward search before the junction entry is limited to the distance
    actually travelled, and a start (almost) at the entry gets None unless a
    crosswalk lies inside the junction. `min_ahead=None` lifts the limit, for
    probing the geometry from the entry itself (distance can then be <= 0).

    Returns a Crossing, or None if there is no junction ahead, no crossing
    ahead of the start or no walk path that stays off the driving lanes.
    """
    entry, travelled = junction_entry_ahead(start_wp, max_distance)
    if entry is None:
        return None

    max_back = 20.0 if min_ahead is None else min(20.0, travelled - min_ahead)
    found = crosswalk_at_junction(entry, crosswalk_index.near(entry.transform.location, 35.0), max_back=max_back)
    if found is None:
        crosswalk, wp, distance = None, entry, travelled
    else:
        crosswalk, along, wp = found
        distance = travelled + along

    path = pedestrian_path(cmap, wp, crosswalk)
    if path is None and crosswalk is not None:
        # The crosswalk ends in the middle of the junction's lanes: use the junction entry instead
        crosswalk, wp, distance = None, entry, travelled
        path = pedestrian_path(cmap, wp)
    if path is None or (min_ahead is not None and distance < min_ahead):
        return None
    spawn, end, walk_distance = path
    return Crossing(wp, distance, crosswalk, spawn, end, walk_distance)


# (metres further back from the road, metres along the road) tried in order when the walker cannot spawn
SPAWN_OFFSETS = ((0.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (1.0, 0.0), (-1.0, 0.0),
                 (0.0, 0.5), (0.0, -0.5), (0.0, 1.0), (0.0, -1.0))


def walker_spawns(cmap, crossing, offsets=SPAWN_OFFSETS):
    """
    Spawn alternatives around `crossing.spawn`, in order of preference:
    yields (offset, transform, walk_distance). The yaw is recomputed towards
    `crossing.end`, which every walk still reaches. Spots on a driving lane
    are skipped.
    """
    spawn, end = crossing.spawn, crossing.end
    dx, dy = end.x - spawn.location.x, end.y - spawn.location.y
    norm = math.hypot(dx, dy) or 1.0
    ux, uy = dx / norm, dy / norm
    for back, side in offsets:
        x = spawn.location.x - ux * back - uy * side
        y = spawn.location.y - uy * back + ux * side
        walk_distance = math.hypot(end.x - x, end.y - y)
        if walk_distance < 1.0:
            continue
        loc = carla.Location(x=x, y=y, z=spawn.location.z)
        if (back, side) != (0.0, 0.0) and cmap.get_waypoint(
                loc, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            continue
        yaw = math.degrees(math.atan2(end.y - y, end.x - x))
        yield (back, side), carla.Transform(loc, carla.Rotation(yaw=yaw)), walk_distance


CROWD_HALF_WIDTH = 1.25  # half width of a painted crosswalk, the same on every map [m]
CROWD_ROW_GAP = 0.7  # distance between two walkers of a crowd queue [m]
CROWD_MIN_MAIN_CLEARANCE = 0.45  # closest a crowd walker's line may be to the main pedestrian's [m]
CROWD_MIN_COLUMN_GAP = 0.4  # closest two columns of the crowd may be [m]
CROWD_DELAY_JITTER = 0.5  # random extra wait of a walker behind another one in its column [s]


@dataclass
class CrowdSpot:
    """
    One crowd pedestrian: `side` "A" starts at the right-hand end of the crossing and walks to the left-hand
    end, "B" the other way round. `spawn` has the yaw towards `end`; `distance` is the walk length [m].

    `column` counts the walker's line across the crosswalk, from 0 (next to the main pedestrian) outwards, and
    `row` its place in the queue of that column, from 0 (the front, the first to leave). Walkers of the same
    side and column share one line.
    """
    side: str
    column: int
    row: int
    spawn: carla.Transform
    end: carla.Location
    distance: float


def _on_driving_lane(cmap, loc):
    return cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is not None


def crowd_spawns(cmap, crossing, count, rng=None, half_width=CROWD_HALF_WIDTH, row_gap=CROWD_ROW_GAP, columns=3,
                 min_gap=0.5, inner_offset=CROWD_MIN_MAIN_CLEARANCE):
    """
    Starts and targets for `count` extra pedestrians crossing at `crossing`, half from each end. Pure geometry,
    so it works on an offline carla.Map.

    The walkers queue up behind the kerb in rows `row_gap` apart (the first row one gap back), `columns` abreast.
    The columns lie between `inner_offset` and `half_width` metres from the main pedestrian's line (the defaults
    give 0.45, 0.85 and 1.25 m: at least 0.4 m apart and clear of the main pedestrian, who walks the centre
    line, with the outer column on the paint of a 2.5 m wide crosswalk). Each stream keeps to its own right, so
    the two pass side by side. Neighbouring columns are shifted half a row against each other, which keeps
    walkers on neighbouring lines about 0.53 m apart instead of 0.4 m.

    Everyone walks a straight line parallel to the main pedestrian's, and walkers of one column share that
    line. They all walk the same distance, so the far end is the near end mirrored: the walker at the front of
    the queue ends furthest out, the one at the back ends at the target point, and the spacing of the queue is
    kept. A walker therefore never has to pass the one ahead of it. Starts and targets are moved outwards off
    the driving lanes; a walker that cannot be placed, or would start closer than `min_gap` to another, is
    moved one row further back or dropped, so fewer than `count` can come back. `rng` (a random.Random) picks
    the columns of a partly filled last row; without it the layout is regular.
    """
    if columns < 1:
        raise ValueError("columns must be at least 1")
    if inner_offset < CROWD_MIN_MAIN_CLEARANCE - 1e-9:
        raise ValueError("inner_offset {:g} m is closer than {:g} m to the main pedestrian's line".format(
            inner_offset, CROWD_MIN_MAIN_CLEARANCE))
    if columns > 1 and (half_width - inner_offset) / (columns - 1) < CROWD_MIN_COLUMN_GAP - 1e-9:
        raise ValueError("{} columns between {:g} and {:g} m are less than {:g} m apart".format(
            columns, inner_offset, half_width, CROWD_MIN_COLUMN_GAP))

    spawn, end = crossing.spawn.location, crossing.end
    dx, dy = end.x - spawn.x, end.y - spawn.y
    norm = math.hypot(dx, dy) or 1.0
    ux, uy = dx / norm, dy / norm  # walk direction, right to left
    rx, ry = -uy, ux  # right of the walk direction, horizontal

    # side -> (queue point at the kerb, ground z there, target point at the far end, ground z there, sign of the
    # walk direction along u); the lateral offset along r has the same sign (each stream keeps to its own right)
    ends = {"A": (spawn, spawn.z - 1.0, end, end.z, 1.0), "B": (end, end.z, spawn, spawn.z - 1.0, -1.0)}
    if columns == 1:
        offsets = [inner_offset]
    else:
        offsets = [inner_offset + (half_width - inner_offset) * c / (columns - 1) for c in range(columns)]

    spots, starts = [], []
    for side, n in (("A", (count + 1) // 2), ("B", count // 2)):
        queue, z0, target, z1, sign = ends[side]
        # Positions in the queue, row by row; a partly filled last row gets random columns
        slots = [(row, col) for row in range(n // columns) for col in range(columns)]
        rest = n % columns
        if rest:
            cols = rng.sample(range(columns), rest) if rng is not None else list(range(rest))
            slots += [(n // columns, col) for col in sorted(cols)]

        # Starts: (column, how far behind the queue point along the walk line, location)
        placed = []
        for row, col in slots:
            lateral = offsets[col] * sign
            for extra in range(3):
                back = (row + 1 + extra) * row_gap + (col % 2) * row_gap / 2.0
                # Back = away from the crosswalk: -u for side A, +u for side B
                start = carla.Location(x=queue.x - ux * sign * back + rx * lateral,
                                       y=queue.y - uy * sign * back + ry * lateral, z=z0)
                if _on_driving_lane(cmap, start):
                    start = _out_of_driving_lane(cmap, start, -ux * sign, -uy * sign)
                    if start is None:
                        continue
                if any(math.hypot(start.x - p.x, start.y - p.y) < min_gap for p in starts):
                    continue
                starts.append(start)
                placed.append((col, -((start.x - queue.x) * ux + (start.y - queue.y) * uy) * sign, start))
                break
        if not placed:
            continue

        # Targets: each column's target point moved outwards off the driving lanes, all by the same amount so the
        # columns keep their relative position
        pushes = {}
        for col in {col for col, _, _ in placed}:
            lateral = offsets[col] * sign
            base = carla.Location(x=target.x + rx * lateral, y=target.y + ry * lateral, z=z1)
            out = _out_of_driving_lane(cmap, base, ux * sign, uy * sign)
            if out is not None:
                pushes[col] = math.hypot(out.x - base.x, out.y - base.y)
        if not pushes:
            continue
        push = max(pushes.values())
        deepest = max(back for _, back, _ in placed)  # the back of the queue ends at the target point

        columns_placed = {}
        for col, back, start in placed:
            if col in pushes:
                columns_placed.setdefault(col, []).append((back, start))
        for col, walkers in sorted(columns_placed.items()):
            lateral = offsets[col] * sign
            for row, (back, start) in enumerate(sorted(walkers, key=lambda walker: walker[0])):
                along = push + deepest - back
                end_loc = carla.Location(x=target.x + rx * lateral + ux * sign * along,
                                         y=target.y + ry * lateral + uy * sign * along, z=z1)
                if _on_driving_lane(cmap, end_loc):
                    continue
                distance = math.hypot(end_loc.x - start.x, end_loc.y - start.y)
                yaw = math.degrees(math.atan2(end_loc.y - start.y, end_loc.x - start.x))
                tf = carla.Transform(carla.Location(x=start.x, y=start.y, z=z0 + 1.0), carla.Rotation(yaw=yaw))
                spots.append(CrowdSpot(side, col, row, tf, end_loc, distance))
    return spots


CROWD_RETRY_BACK = (0.25, 0.5)  # how far further back a crowd walker is tried when its start cannot be spawned [m]


def crowd_spawn_retries(spot, back=CROWD_RETRY_BACK):
    """
    Alternative starts for a crowd walker that could not be spawned at `spot`: the same line, the same target
    and the same column, but further back from the road by each of `back` metres (a prop or another actor often
    blocks the exact spot). Yields (metres back, CrowdSpot) with the spawn moved and the walk distance longer by
    that amount. The queue order is unchanged, as the row and the target are the same.
    """
    tf = spot.spawn
    fwd = tf.get_forward_vector()  # the walk direction; back is away from the road
    for extra in back:
        loc = carla.Location(x=tf.location.x - fwd.x * extra, y=tf.location.y - fwd.y * extra, z=tf.location.z)
        yield extra, replace(spot, spawn=carla.Transform(loc, tf.rotation), distance=spot.distance + extra)


def crowd_timing(spots, rng, max_delay, speed_range, row_gap=CROWD_ROW_GAP):
    """
    Start delay [s] and walking speed [m/s] for each of `spots`, as a list of (delay, speed) in the same order.

    Walkers of one column share a line and one speed, drawn from `speed_range` (min, max). The front of the
    queue waits a random 0 to `max_delay` s after the trigger; every walker behind it waits at least
    `row_gap / min speed` s longer than the one ahead (plus a little random extra), so it never closes the gap
    to it: it starts only when the one ahead has moved on by a row, and walks no faster. Columns are independent
    (parallel lines). `rng` is a random.Random.
    """
    min_speed, max_speed = min(speed_range), max(speed_range)
    if min_speed <= 0.0:
        raise ValueError("min_speed must be positive")
    groups = {}
    for i, spot in enumerate(spots):
        groups.setdefault((spot.side, spot.column), []).append(i)

    timing = [None] * len(spots)
    for key in sorted(groups):
        speed = rng.uniform(min_speed, max_speed)
        delay = rng.uniform(0.0, max_delay)
        for rank, i in enumerate(sorted(groups[key], key=lambda i: spots[i].row)):
            if rank:
                delay += row_gap / min_speed + rng.uniform(0.0, CROWD_DELAY_JITTER)
            timing[i] = (delay, speed)
    return timing


# Sizes of the real blueprints (half the bounding box, read from CARLA 0.9.16): bicycles are 0.29 to 0.43 m
# (bh.crossbike) wide and 0.75 to 0.92 m half long, walkers 0.19 to 0.25 m wide. Rounded up.
MAX_BIKE_HALF_WIDTH = 0.45  # [m]
MAX_BIKE_HALF_LENGTH = 0.95  # [m]
MAX_WALKER_HALF_WIDTH = 0.25  # [m]
CYCLE_CLEARANCE = 0.15  # free space kept between two bodies side by side [m]

# The two cycle lanes: the first clears the outer column of the crowd (CROWD_HALF_WIDTH from the main pedestrian's
# line) by a walker, a bicycle and the clearance, the second clears a bicycle in the first one: 2.10 and 3.15 m
CYCLE_FIRST_LANE = CROWD_HALF_WIDTH + MAX_WALKER_HALF_WIDTH + MAX_BIKE_HALF_WIDTH + CYCLE_CLEARANCE
CYCLE_SECOND_LANE = CYCLE_FIRST_LANE + 2.0 * MAX_BIKE_HALF_WIDTH + CYCLE_CLEARANCE
CYCLE_LANE_OFFSETS = (CYCLE_FIRST_LANE, CYCLE_SECOND_LANE)  # distances of the two cycle lanes from the main pedestrian's line [m]
CYCLE_MIN_MAIN_CLEARANCE = 1.6  # closest a cyclist may come to the main pedestrian's line [m]
CYCLE_GAP = 2.5  # distance between two bicycles of a queue [m]
CYCLE_START_MARGIN = 1.2  # start this far inside the first driving lane point, so the whole bicycle is on the road [m]
CYCLE_END_MARGIN = MAX_BIKE_HALF_LENGTH + 0.5  # end this far inside the last driving lane point: the longest bicycle clears the kerb [m]
CYCLE_MIN_RIDE = 2.0  # shortest ride worth making [m]
CYCLE_DECELERATION = 4.0  # assumed braking of a bicycle at full brake, deliberately low (not measured) [m/s^2]
CYCLE_STEP = 1.0 / 20.0  # simulation step assumed when the world has no fixed one (the default frame rate) [s]
CYCLE_MIN_FORCED_RIDE = 0.5  # shortest part of a ride that is pushed at a forced speed [m]
CYCLE_DELAY_JITTER = 0.5  # random extra wait of a bicycle behind another one in its lane [s]
CYCLE_MIN_SPACING = 0.8  # bicycles of a lane stay at least this fraction of the gap apart, also after shifting a ride
_CYCLE_SCAN_STEP = 0.25  # resolution of the search for the edges of the road along a cycle lane [m]
_CYCLE_SHIFTS = (0.0, 0.25, -0.25, 0.5, -0.5, 0.75, -0.75, 1.0, -1.0)  # tried when a start or end has no driving lane [m]


@dataclass
class CyclistSpot:
    """
    One cyclist: `side` "A" rides in the direction of the main pedestrian (from the ego's right to its left), "B"
    the other way round. `spawn` has the yaw towards `end`; `distance` is the ride length [m]. `offset` is the
    distance of the cycle lane from the main pedestrian's line [m]. `row` is the place in the queue of the lane, 0
    for the front (the first to leave).
    """
    side: str
    row: int
    offset: float
    spawn: carla.Transform
    end: carla.Location
    distance: float


def _driving_z(cmap, loc):
    """Road surface height at `loc` if it is on a driving lane, else None."""
    wp = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving)
    return None if wp is None else wp.transform.location.z


def cyclist_spawns(cmap, crossing, count, lane_offsets=CYCLE_LANE_OFFSETS, gap=CYCLE_GAP,
                   start_margin=CYCLE_START_MARGIN, end_margin=CYCLE_END_MARGIN, min_ride=CYCLE_MIN_RIDE):
    """
    Starts and targets for `count` bicycles crossing at `crossing`, half from each side. Pure geometry, so it
    works on an offline carla.Map.

    The bicycles ride in a cycle-crossing lane beside the zebra, on the junction side of the crosswalk (the side
    the ego drives towards), parallel to the main pedestrian's line and clear of the crowd, which keeps within
    1.25 m of that line. There are two such lanes, `lane_offsets` metres from the line (2.1 and 3.15 m: wide enough
    for the widest bicycle beside the outer crowd column and beside a bicycle of the other lane, see
    MAX_BIKE_HALF_WIDTH), one per direction; each direction keeps to its own right, so the outer lane belongs to the
    stream whose right-hand side faces away from the zebra.

    A bicycle starts ON THE ROAD, `start_margin` metres inside the first driving lane point met along its lane
    (just inside the kerb, so it does not drop off it), and ends `end_margin` metres inside the last one (just
    before the far kerb; the default leaves room for half the longest bicycle and 0.5 m more). Bicycles of one lane queue `gap` metres apart, the front one starting furthest in; all
    ride the same distance, so the queue is mirrored at the far end and none has to pass another. A lane takes
    as many of its bicycles as fit with rides of at least `min_ride` metres, so fewer than `count` can come
    back. Side A gets (count + 1) // 2 bicycles, side B count // 2.

    Returns a list of CyclistSpot.
    """
    spawn, end = crossing.spawn.location, crossing.end
    dx, dy = end.x - spawn.x, end.y - spawn.y
    norm = math.hypot(dx, dy) or 1.0
    ux, uy = dx / norm, dy / norm  # main pedestrian's walk direction
    rx, ry = -uy, ux  # right of the walk direction
    fwd = crossing.wp.transform.get_forward_vector()
    sign = 1.0 if rx * fwd.x + ry * fwd.y >= 0.0 else -1.0  # the junction side is along sign * r
    inner, outer = min(lane_offsets), max(lane_offsets)
    # Right-hand traffic: side A walks along u, its right is +r; side B walks against u, its right is -r. The stream
    # whose right is on the junction side's far side (away from the zebra) takes the outer lane
    offsets = {"A": outer if sign > 0 else inner, "B": outer if sign < 0 else inner}

    spots = []
    for side, n in (("A", (count + 1) // 2), ("B", count // 2)):
        if n <= 0:
            continue
        offset = offsets[side]
        ox, oy = rx * sign * offset, ry * sign * offset

        def point(t):
            return carla.Location(x=spawn.x + ux * t + ox, y=spawn.y + uy * t + oy, z=spawn.z - 1.0)

        # Where the cycle lane has road under it: first and last driving lane point along the main line's extent
        steps = int(math.ceil((norm + 4.0) / _CYCLE_SCAN_STEP))
        on_road = [k * _CYCLE_SCAN_STEP - 2.0 for k in range(steps + 1)
                   if _driving_z(cmap, point(k * _CYCLE_SCAN_STEP - 2.0)) is not None]
        if not on_road:
            continue
        t_first, t_last = on_road[0] + start_margin, on_road[-1] - end_margin
        free = (t_last - t_first) - min_ride
        if free < 0.0:
            continue
        n = min(n, int(free / gap) + 1)
        ride = (t_last - t_first) - (n - 1) * gap

        extent = t_last - t_first
        taken = []  # progress of the bicycles of this lane placed so far
        for row in range(n):
            # Progress along the ride direction at which this bicycle starts: the front one furthest in. A point
            # on a seam between two lane pieces can have no driving lane, so shift the whole ride a little if need be
            back = (n - 1 - row) * gap
            for shift in _CYCLE_SHIFTS:
                progress = back + shift
                if progress < 0.0 or progress + ride > extent + 1e-9:
                    continue
                if any(abs(progress - other) < gap * CYCLE_MIN_SPACING for other in taken):
                    continue
                if side == "A":
                    t0, t1 = t_first + progress, t_first + progress + ride
                else:
                    t0, t1 = t_last - progress, t_last - progress - ride
                a, b = point(t0), point(t1)
                za, zb = _driving_z(cmap, a), _driving_z(cmap, b)
                if za is None or zb is None:
                    continue
                taken.append(progress)
                yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
                tf = carla.Transform(carla.Location(x=a.x, y=a.y, z=za + 0.1), carla.Rotation(yaw=yaw))
                spots.append(CyclistSpot(side, row, offset, tf, carla.Location(x=b.x, y=b.y, z=zb),
                                         math.hypot(b.x - a.x, b.y - a.y)))
                break
    return spots


def cyclist_forced_ride(distance, speed, step=CYCLE_STEP, deceleration=CYCLE_DECELERATION):
    """
    How far a bicycle that must stop within `distance` metres may be pushed at its forced `speed` [m]: the ride
    minus what it still travels before the push ends (the behaviour notices the end one step late, so `speed` *
    `step`) and its braking distance (`speed`^2 / (2 `deceleration`)). Never negative.
    """
    return max(distance - speed * step - speed * speed / (2.0 * deceleration), 0.0)


def cyclist_speed_limit(distance, step=CYCLE_STEP, deceleration=CYCLE_DECELERATION, min_ride=CYCLE_MIN_FORCED_RIDE):
    """
    Highest speed [m/s] at which a bicycle can still be pushed for `min_ride` metres and stopped within `distance`:
    the positive root of speed * step + speed^2 / (2 deceleration) = distance - min_ride. A short ride caps the
    speed instead of being stretched past the kerb.
    """
    room = max(distance - min_ride, 0.0)
    drift = deceleration * step
    return math.sqrt(drift * drift + 2.0 * deceleration * room) - drift


def cyclist_timing(spots, rng, max_delay, speed_range, gap=CYCLE_GAP, speed_limits=None):
    """
    Start delay [s] and riding speed [m/s] for each of `spots`, as a list of (delay, speed) in the same order.

    The bicycles of one lane share one speed, drawn from `speed_range` (min, max), and ride the same distance.
    `speed_limits` (a list in the order of `spots`, see cyclist_speed_limit) caps the speed of a lane at the lowest
    limit among its bicycles; the delays use the capped speed, so the queue still keeps its gap.
    The front of the queue waits a random 0 to `max_delay` s after the trigger; every bicycle behind it waits
    `gap / speed` s (plus a little random extra) longer than the one ahead, so it starts only when the one ahead
    has moved on by a gap and then rides no faster: nobody catches up. The two lanes are independent. `rng` is a
    random.Random.
    """
    min_speed, max_speed = min(speed_range), max(speed_range)
    if min_speed <= 0.0:
        raise ValueError("min_speed must be positive")
    groups = {}
    for i, spot in enumerate(spots):
        groups.setdefault(spot.side, []).append(i)

    timing = [None] * len(spots)
    for key in sorted(groups):
        speed = rng.uniform(min_speed, max_speed)
        if speed_limits is not None:
            speed = min(speed, min(speed_limits[i] for i in groups[key]))
        delay = rng.uniform(0.0, max_delay)
        for rank, i in enumerate(sorted(groups[key], key=lambda i: spots[i].row)):
            if rank:
                delay += gap / speed + rng.uniform(0.0, CYCLE_DELAY_JITTER)
            timing[i] = (delay, speed)
    return timing


# --- Where the cyclists wait: the sidewalk beyond the kerb -------------------------------------------------------------

SIDEWALK_BEYOND = 1.5  # the first waiting bicycle stands this far beyond the kerb, on the cycle line [m]
SIDEWALK_SEARCH = 1.5  # a Sidewalk lane is looked for this far beyond the first waiting distance [m]
SIDEWALK_QUEUE_GAP = 2.2  # waiting bicycles queue this far apart: the longest one (1.9 m) plus a little [m]
SIDEWALK_RETRY_BACK = (0.5, 1.0)  # how far further back a waiting spot is tried when the bicycle cannot be spawned [m]
SIDEWALK_LIFT = 0.15  # height of the waiting spot over the road when the map has no sidewalk lane there [m]
_KERB_SCAN_STEP = 0.1  # resolution of the search for the kerb along a cycle line [m]
_KERB_SEAM = 0.6  # a stretch of the line without a driving lane shorter than this is a seam in the road, not the kerb [m]
_KERB_MAX_SCAN = 60.0  # the road is crossed within this distance along the line [m]


def _kerb_distance(cmap, start, fx, fy):
    """
    How far back (against the ride direction (fx, fy)) from `start`, a point on a driving lane, the road ends along
    the line: the distance to the last driving lane point before a stretch of at least _KERB_SEAM metres without one.
    """
    last = 0.0
    k = 0.0
    while k - last < _KERB_SEAM and k < _KERB_MAX_SCAN:
        k += _KERB_SCAN_STEP
        probe = carla.Location(x=start.x - fx * k, y=start.y - fy * k, z=start.z)
        if _driving_z(cmap, probe) is not None:
            last = k
    return last


def _on_sidewalk(cmap, loc):
    return cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Sidewalk) is not None


def _waiting_point(cmap, spot, back):
    """
    The waiting point `back` metres behind the start of `spot` along its line as a Transform that faces the ride
    direction, or None when the bicycle there (centre, nose and tail) would touch a driving lane. The z is the
    height of the lane under the point (the sidewalk, else the shoulder or whatever the map has there), else the
    start's height plus SIDEWALK_LIFT.
    """
    start = spot.spawn.location
    fwd = spot.spawn.get_forward_vector()
    for along in (0.0, MAX_BIKE_HALF_LENGTH, -MAX_BIKE_HALF_LENGTH):
        probe = carla.Location(x=start.x - fwd.x * (back - along), y=start.y - fwd.y * (back - along), z=start.z)
        if _on_driving_lane(cmap, probe):
            return None
    loc = carla.Location(x=start.x - fwd.x * back, y=start.y - fwd.y * back, z=start.z)
    road_z = start.z - 0.1  # spot.spawn is 0.1 m over the road
    lane = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Sidewalk)
    if lane is None:
        lane = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
    if lane is not None and abs(lane.transform.location.z - road_z) < 1.0:
        z = lane.transform.location.z
    else:
        z = road_z + SIDEWALK_LIFT
    return carla.Transform(carla.Location(x=loc.x, y=loc.y, z=z), spot.spawn.rotation)


def cyclist_waiting_spots(cmap, spot, count=1, beyond=SIDEWALK_BEYOND, gap=SIDEWALK_QUEUE_GAP):
    """
    Where bicycles wait to ride the cycle line of `spot` (a CyclistSpot): on the sidewalk just beyond the kerb they
    start from, on the same line, facing the way they will ride. Pure geometry, so it works on an offline carla.Map.

    The kerb is the end of the road along the line behind the start of `spot` (a seam shorter than 0.6 m inside the
    road does not count). Position 0 is about `beyond` metres beyond it: where the map has a Sidewalk lane on the line
    within SIDEWALK_SEARCH metres further out (CARLA puts a shoulder of about a metre first, on some maps), the
    nearest such point from `beyond` outwards, else exactly `beyond` metres. Each further position is `gap` metres
    further back, so bicycles of the queue (the longest is 1.9 m) stay clear of each other. The queue ends at the
    first position where the bicycle would touch a driving lane (a side street, a bend of the road), so fewer than
    `count` spots can come back, and none when even the first does not fit. Returns a list of carla.Transform,
    index = queue position, 0 = the front one (nearest the kerb, the first to leave); the z is the height of the
    lane under the point.
    """
    start = spot.spawn.location
    fwd = spot.spawn.get_forward_vector()
    kerb = _kerb_distance(cmap, start, fwd.x, fwd.y)
    base = kerb + beyond
    for step in range(int(SIDEWALK_SEARCH / _KERB_SCAN_STEP) + 1):
        back = base + step * _KERB_SCAN_STEP
        probe = carla.Location(x=start.x - fwd.x * back, y=start.y - fwd.y * back, z=start.z)
        if _on_sidewalk(cmap, probe) and _waiting_point(cmap, spot, back) is not None:
            base = back
            break
    spots = []
    for position in range(max(count, 0)):
        tf = _waiting_point(cmap, spot, base + position * gap)
        if tf is None:
            break
        spots.append(tf)
    return spots


def cyclist_wait_retries(cmap, tf, back=SIDEWALK_RETRY_BACK):
    """
    Alternative waiting points for a bicycle that could not be spawned at `tf` (a Transform from
    cyclist_waiting_spots): the same line, `back` metres further back. Yields (metres back, Transform), leaving out
    the ones that would touch a driving lane.
    """
    fwd = tf.get_forward_vector()
    for extra in back:
        loc = carla.Location(x=tf.location.x - fwd.x * extra, y=tf.location.y - fwd.y * extra, z=tf.location.z)
        if any(_on_driving_lane(cmap, carla.Location(x=loc.x + fwd.x * along, y=loc.y + fwd.y * along, z=loc.z))
               for along in (0.0, MAX_BIKE_HALF_LENGTH, -MAX_BIKE_HALF_LENGTH)):
            continue
        yield extra, carla.Transform(loc, tf.rotation)


CYCLE_EXIT_MAX = 10.0  # a bicycle rolls at most this far from the end of its ride to the far sidewalk [m]


def cyclist_exit_spot(cmap, spot, max_distance=CYCLE_EXIT_MAX):
    """
    Where a bicycle that has ridden `spot` (a CyclistSpot) leaves to: the waiting point of the opposite direction on
    the same line (cyclist_waiting_spots of the ride reversed), on the sidewalk beyond the far kerb, as a Transform
    that keeps the ride's heading. None when the far side has no such point (a driving lane or nothing in the way),
    or when it is more than `max_distance` metres past the end of the ride (a ride that stops far from the kerb: a
    wide road or a median, where the walk line is shorter than the road).
    """
    end = spot.end
    back = CyclistSpot(spot.side, spot.row, spot.offset,
                       carla.Transform(carla.Location(x=end.x, y=end.y, z=end.z + 0.1),
                                       carla.Rotation(yaw=spot.spawn.rotation.yaw + 180.0)),
                       spot.spawn.location, spot.distance)
    waits = cyclist_waiting_spots(cmap, back, 1)
    if not waits or math.hypot(waits[0].location.x - end.x, waits[0].location.y - end.y) > max_distance:
        return None
    return carla.Transform(waits[0].location, carla.Rotation(yaw=spot.spawn.rotation.yaw))


# --- When something may appear or vanish: out of the ego's sight -------------------------------------------------------

HIDE_DISTANCE = 50.0  # from this far away, a point off to the side of the ego is out of sight [m]
HIDE_BEHIND_DISTANCE = 15.0  # a point this close or closer is never hidden [m]
HIDE_BEHIND_ANGLE = 110.0  # farther than the behind distance, a point more than this far off the ego's heading is behind it [deg]
HIDE_FAR_DISTANCE = 120.0  # from this far away, anything is out of sight, straight ahead included [m]
HIDE_SIDE_ANGLE = 60.0  # from hide_distance on, a point more than this far off the ego's heading is out of sight [deg]
HIDE_REVERSE_SPEED = -0.5  # the ego reverses when its speed along its heading is below this [m/s]


def hidden_from_ego(ego_transform, location, hide_distance=HIDE_DISTANCE, behind_distance=HIDE_BEHIND_DISTANCE,
                    behind_angle=HIDE_BEHIND_ANGLE, far_distance=HIDE_FAR_DISTANCE, side_angle=HIDE_SIDE_ANGLE,
                    forward_speed=None):
    """
    True when the driver of the ego cannot see something at `location` appear or vanish. The distance is 2D and the
    angle is the one between the ego's forward vector and the direction to `location`. Hidden when
      (a) the distance is at least `far_distance`, whatever the direction, or
      (b) the distance is at least `hide_distance` and the angle is more than `side_angle` (a point 50 to 120 m
          straight ahead is in plain view of a chase camera, one off to the side is not), or
      (c) the distance is at least `behind_distance` and the angle is more than `behind_angle` (behind the car).
    While the ego is reversing (`forward_speed`, the ego's velocity along its heading [m/s], None when unknown, is
    below HIDE_REVERSE_SPEED) the driver looks backwards: (c) does not apply, and (b) only does up to `behind_angle`,
    so it stays a rule for points to the side (a point 50 to 120 m straight behind is in view, as one straight ahead
    is); (a) holds whatever the ego does.
    Every spawn and removal that happens while a scenario runs follows this rule.
    """
    ego = ego_transform.location
    dx, dy = location.x - ego.x, location.y - ego.y
    distance = math.hypot(dx, dy)
    if distance >= far_distance:
        return True
    if distance < behind_distance or distance <= 0.0:
        return False
    yaw = math.radians(ego_transform.rotation.yaw)
    cos_angle = (dx * math.cos(yaw) + dy * math.sin(yaw)) / distance
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))
    reversing = forward_speed is not None and forward_speed < HIDE_REVERSE_SPEED
    if distance >= hide_distance and angle > side_angle and not (reversing and angle > behind_angle):
        return True
    return angle > behind_angle and not reversing


@dataclass(frozen=True)
class HideRule:
    """
    The numbers of hidden_from_ego, so one object can be handed around (the <spawning> block sets them). hidden() takes
    the ego's forward speed [m/s]: while it reverses, nothing behind the car is hidden short of far_distance, and
    hide_distance only hides points to the side (angle up to behind_angle).
    """
    hide_distance: float = HIDE_DISTANCE
    behind_distance: float = HIDE_BEHIND_DISTANCE
    behind_angle: float = HIDE_BEHIND_ANGLE
    far_distance: float = HIDE_FAR_DISTANCE
    side_angle: float = HIDE_SIDE_ANGLE

    def hidden(self, ego_transform, location, forward_speed=None):
        return hidden_from_ego(ego_transform, location, self.hide_distance, self.behind_distance, self.behind_angle,
                               self.far_distance, self.side_angle, forward_speed)


OCCLUDER_CLEAR = 1.5  # free space between two vehicles of the occluder's queue [m]
PROP_MARGIN = 0.3  # free space between the occluder's prop and the crosswalk [m]
PROP_BEYOND = 1.1  # how far the prop reaches past the pedestrian's start, away from the road [m]


def _near_edge_back(crossing):
    """How far the near edge of the crosswalk is behind the crossing point along the ego lane [m]; 0 without a crosswalk."""
    if crossing.crosswalk is None:
        return 0.0
    fwd = crossing.wp.transform.get_forward_vector()
    loc = crossing.wp.transform.location
    return max(0.0, -min((p.x - loc.x) * fwd.x + (p.y - loc.y) * fwd.y for p in crossing.crosswalk.points))


def occluder_transforms(crossing, half_lengths, gap, clear=OCCLUDER_CLEAR):
    """
    Where a queue of vehicles stands in the lane right of the ego lane, before the crosswalk of `crossing`: one
    carla.Transform per entry of `half_lengths` (half length of each vehicle [m], the first one nearest the
    crosswalk). The front of the first is `gap` m before the near edge of the crosswalk (the crossing point when
    there is no crosswalk), the others follow with `clear` m of free space between them. Each one's centre is looked
    up `s` m back along the ego lane, then one lane to the right. Raises ValueError when that is inside the junction
    or not a driving lane in the ego's direction.
    """
    wp = crossing.wp
    transforms = []
    back = _near_edge_back(crossing) + gap  # how far behind the crossing point the front of the next vehicle is
    for half in half_lengths:
        lane = lane_point(wp, -(back + half))
        kerb = lane.get_right_lane()
        if (lane.is_junction or kerb is None or kerb.lane_type != carla.LaneType.Driving
                or (kerb.lane_id > 0) != (wp.lane_id > 0)):
            raise ValueError("No driving lane right of the ego lane before the crosswalk (or the junction starts too close "
                             "to it): the ego must be in the lane left of the kerb lane; pick a start with "
                             "apps/find_crossing_spots.py --lane inner")
        transforms.append(kerb.transform)
        back += 2.0 * half + clear
    return transforms


def walk_in_spawns(cmap, crossing, walk_in, offsets=SPAWN_OFFSETS):
    """
    walker_spawns for a pedestrian that first walks `walk_in` m along the road, in the ego's direction, to the spot
    walker_spawns gives, then crosses: yields (offset, start transform, crossing transform, walk_distance). The start is
    `walk_in` m back from the crossing spot (its yaw is the walk along the road) and skipped when it is on a driving lane;
    the crossing transform and walk_distance are walker_spawns's.
    """
    fwd = crossing.wp.transform.get_forward_vector()
    yaw = math.degrees(math.atan2(fwd.y, fwd.x))
    for offset, cross, walk_distance in walker_spawns(cmap, crossing, offsets):
        loc = carla.Location(x=cross.location.x - fwd.x * walk_in, y=cross.location.y - fwd.y * walk_in,
                             z=cross.location.z)
        if cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is None:
            yield offset, carla.Transform(loc, carla.Rotation(yaw=yaw)), cross, walk_distance


def prop_transform(crossing, half_length, half_width, margin=PROP_MARGIN, beyond=PROP_BEYOND):
    """
    Centre of a box of that half length (along its own forward axis, which lies along the road) and half width that
    hides the pedestrian's start (crossing.spawn) from the ego lane: on the kerb side, its crosswalk end `margin` m
    before the near edge of the crosswalk and its far side `beyond` m past the start. Long and close to the start, the
    box is in the way of every line of sight from the ego lane to it. The transform is at the height of the start's
    ground and faces along the ego lane.
    """
    tf = crossing.wp.transform
    fwd, right, loc = tf.get_forward_vector(), tf.get_right_vector(), tf.location
    start = crossing.spawn.location
    across = (start.x - loc.x) * right.x + (start.y - loc.y) * right.y + beyond - half_width
    along = -(_near_edge_back(crossing) + margin + half_length)
    return carla.Transform(carla.Location(x=loc.x + fwd.x * along + right.x * across,
                                          y=loc.y + fwd.y * along + right.y * across, z=start.z - 1.0),
                           carla.Rotation(yaw=tf.rotation.yaw))


# m/s^2: a CARLA walker reaches its walking speed from rest at about this rate (live), losing speed / (2 a) seconds
WALKER_ACCELERATION = 2.0


def impact_point(crossing, ego_half_width, overlap, ego_half_length=0.0, start=None):
    """
    Where a pedestrian walking from `start` (default crossing.spawn) to crossing.end meets the ego if neither brakes,
    for `overlap` percent: the share of the ego's width from the nearside (the side the pedestrian comes from) to the
    point of the front it hits, so 50 is the centre line, 25 is nearer the pedestrian's side, 75 farther. This is
    the CPNCO definition (".agents/plans/ncap-data-review.md" 2.1 and 2.3: struck at 25/50/75 % of the VUT width when
    no braking is applied; P0-3: nearside, lateral shift (50 - overlap) % of the width).
    The impact is the point of the walk line that lies that far right of the ego route at crossing.wp (the lane's
    centre line there). Returns (impact, aim): both carla.Location, `aim` is on that line `ego_half_length` before
    the impact, where the ego's centre is when its front arrives.
    """
    tf = crossing.wp.transform
    f, r, o = tf.get_forward_vector(), tf.get_right_vector(), tf.location
    a = crossing.spawn.location if start is None else start
    dx, dy = crossing.end.x - a.x, crossing.end.y - a.y
    norm = math.hypot(dx, dy)
    ux, uy = dx / norm, dy / norm
    lateral = (50.0 - overlap) / 100.0 * 2.0 * ego_half_width  # right of the route's centre line [m]
    # the walk reaches that lateral position after s m: (a - o + u * s) . r = lateral
    s = (lateral - ((a.x - o.x) * r.x + (a.y - o.y) * r.y)) / (ux * r.x + uy * r.y)
    px, py = a.x + ux * s, a.y + uy * s
    along = (px - o.x) * f.x + (py - o.y) * f.y - ego_half_length
    return carla.Location(x=px, y=py, z=o.z), carla.Location(x=o.x + f.x * along, y=o.y + f.y * along, z=o.z)
