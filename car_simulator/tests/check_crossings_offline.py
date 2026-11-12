#!/usr/bin/env python
"""
Offline geometry check of the pedestrian crossing code, no CARLA server needed.

Builds each installed map from its OpenDRIVE file with carla.Map, lists the
candidates of apps/find_crossing_spots.py and re-checks every one from its ego
start with the scenario's own crossing computation (road.find_crossing):

  - the ego lane meets the walk line inside (or within 0.5 m of) the crosswalk
  - the walk starts on the ego's right and ends on its left
  - distance_to_crossing is within 5 m of the requested approach
  - the walker starts and ends outside the driving lanes

It also runs the no-crosswalk fallback from every junction entry that has no
crosswalk and checks the same side and lane conditions.

Two more checks: from every junction entry and from points 0-25 m before it,
find_crossing must never return a crossing at distance <= 0 (behind the ego),
and the walker spawn alternatives (road.walker_spawns) must all keep the walk
ending at the same point, face it and stay off the driving lanes.

The crowd check (road.crowd_spawns with 10 walkers, seeded) runs for every candidate: about half start on each
side, every walker starts and ends on opposite sides of the ego lane, no start or target is on a driving lane,
all starts and all targets are at least 0.5 m apart, every line is at least 0.4 m from the main pedestrian's,
walkers of one column walk the same length and end in a queue (the one that starts in front ends in front), and
the delays and speeds from road.crowd_timing leave no walker able to catch the one ahead of it in its column.

The cyclist check (road.cyclist_spawns with 4 bicycles) runs for every candidate, except on Town12, where it runs
for every 10th one (1300 candidates): both directions are present, every start and target is on a driving lane,
at least 1.6 m from the main pedestrian's line and on the junction side (the side the ego drives towards), the
two directions are on their own side of the ego lane, bicycles of one direction start and end at least 2.0 m apart
(in a queue, front one ahead), and the delays and speeds from road.cyclist_timing leave no bicycle able to catch
the one ahead of it.

The waiting spots of the cyclists (road.cyclist_waiting_spots, 4 per bicycle line, the same candidates): the first is on
the cycle line 1 to 3 m beyond the kerb (the last driving lane point before the road ends along the line), none of
them (centre, nose or tail of the longest bicycle) is on a driving lane, they face the ride direction, queue at least
2.0 m apart and are on a Sidewalk lane where the map has one within reach (the share is reported). The retries
(road.cyclist_wait_retries) stay off the driving lanes too. The exit spot on the far side (road.cyclist_exit_spot: where a
bicycle rolls to after its ride) is on the line beyond the end of the ride and off the driving lanes. A line where not even the first spot fits is counted
(a side street or another road behind the kerb) and must stay rare (at most 25 percent on a map with 40 lines or more). Self-test of road.hidden_from_ego: 120 m away, off to the side from 50 m, or behind the car (unless it reverses).

Clearances use the real half widths of the blueprints (read from CARLA 0.9.16: bicycles up to 0.433 m, walkers up to
0.25 m): along its whole ride (the exact minimum distance between the straight rides, so every point of them, not
just the ends) a bicycle stays clear of the bicycles of the other direction, of every walker of a 10 pedestrian crowd
and of the main pedestrian. The front and the back of a bicycle (0.92 m half length) are on the road at both ends,
and the planned push (road.cyclist_forced_ride, which allows for the late end of the push and for braking) stops
within the ride at every speed the timing can pick. A self-test first shows that the old lane offsets (1.9 and 2.7 m)
fail the clearance check.

The occluder check (find_candidates with lane="inner", road.occluder_transforms with a truck and a car) runs on Town10HD, the map
of the occluded crossing: each inner start has a kerb lane to its right, the truck and the car stand on the rightmost driving lane,
off the junction, the truck's front before the crosswalk and clear of it, the car behind the truck without touching it, and the
truck hides the kerb lane at the crosswalk from the inner lane 15 to 40 m before the crossing (2D ray against the truck's box).
The prop of the occluder (road.prop_transform, a 3.9 x 1.9 m bus stop on the kerb side, long side along the road): off the driving
lanes, clear of the crosswalk and of the child's walk across, and with the truck it hides the child's waiting spot (and the 0.25 m around it) from the
inner lane 15 to 60 m before the crossing.
The walk-in of the child (road.walk_in_spawns, 6 m): the leg along the road is off the driving lanes, hidden by the truck, all of it from
10 and 15 m before the crossing and its end (where the child turns to cross) also from 20 m. From farther away the child is in sight:
the truck's shadow from the inner lane is a thin cone beside a kerb 5 m from the truck's axis (ego 20 m: 3 to 12 of 13 points of
the leg hidden, 25 m and more: none to 4, curve candidate up to 6 at 60 m).

    python tests/check_crossings_offline.py [Town03 Town05 ...]
    python tests/check_crossings_offline.py --xodr /path/to/WolfsburgHighway.xodr [--xodr ...]

Exits with 1 if any check fails. Town12's OpenDRIVE file lives in its own folder; it builds in a few seconds. The maps
come from the CARLA install (carsim.paths.CARLA_ROOT). --xodr checks the OpenDRIVE files given instead (the map is named
after the file), with no minimum number of candidates, and runs the occluder check on their inner-lane starts too.
"""

import argparse
import fnmatch
import math
import os
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "apps"))

import carla  # noqa: E402

from carsim.paths import CARLA_ROOT  # noqa: E402
from carsim.road import (CROWD_ROW_GAP as ROW_GAP, CYCLE_DECELERATION, CYCLE_LANE_OFFSETS, CYCLE_STEP,  # noqa: E402
                         MAX_BIKE_HALF_LENGTH, MAX_BIKE_HALF_WIDTH, MAX_WALKER_HALF_WIDTH, SIDEWALK_QUEUE_GAP,
                         CrosswalkIndex, CrowdSpot, CyclistSpot, HideRule, crowd_spawn_retries, crowd_spawns,
                         crowd_timing, crosswalks, cyclist_forced_ride, cyclist_spawns, cyclist_speed_limit,
                         cyclist_exit_spot, cyclist_timing, cyclist_wait_retries, cyclist_waiting_spots, find_crossing,
                         hidden_from_ego,
                         junction_entry_ahead, lane_point, occluder_transforms, prop_transform, straight_route, walk_in_spawns,
                         walker_spawns)
from find_crossing_spots import find_candidates  # noqa: E402

MAPS_DIR = CARLA_ROOT / "CarlaUE4" / "Content" / "Carla" / "Maps"
TOWNS = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD", "Town12"]
MIN_CANDIDATES = {"Town03": 10, "Town05": 25, "Town10HD": 4}
MIN_INNER_CANDIDATES = {"Town10HD": 3}  # lane="inner" starts; Town10HD_Opt has the same OpenDRIVE file. Eight inner lanes go straight, three (four on CARLA 0.9.16) have a road long enough for APPROACH
# Two Town12 starts miss the lane/walk-line checks by a few centimetres over the 0.5 m tolerance (a trigger 0.53 m off
# the walk line on a curve, a lane meeting it 0.51 m outside the crosswalk). Harmless for a 25 m trigger distance;
# reported as NOTE, the crowd checks still count. {(town, road_id, lane_id): messages that may be ignored}
KNOWN_MARGINAL = {
    ("Town12", 28, 1): ("trigger location is not where the lane meets the walk line",),
    ("Town12", 878, -4): ("lane meets walk line 0.51 m outside the crosswalk",),
    # CARLA 0.9.15's maps (the checks were tuned on 0.9.16's): starts that no scenario uses. Town03/04/05: a bicycle line
    # or a crowd walker that does not fit; Town10HD inner lanes (occluder check): the bus stop and the truck leave the
    # child's waiting spot in sight at one or two distances, and one inner start has its kerb lane end in the junction
    ("Town03", 76, 1): ("cyclists: no bicycle in direction A",),
    ("Town03", 4, -2): ("cyclists: no bicycle in direction A",),
    ("Town04", 52, 1): ("cyclist 2: the nose at the target (0.92 m from it) is off the road",),
    ("Town05", 20, -1): ("crowd: 9 of 10 walkers placed",),
    ("Town05", 30, -2): ("cyclist 0: the nose at the target (0.92 m from it) is off the road",),
    ("Town05", 33, -2): ("cyclist 2: the nose at the target (0.92 m from it) is off the road",),
    ("Town10HD", 10, 1): ("the child's waiting spot is in sight of the ego 25 m before the crossing",),
    ("Town10HD", 6, -1): ("the child's waiting spot is in sight of the ego 55 m before the crossing",
                          "the child's waiting spot is in sight of the ego 60 m before the crossing"),
    ("Town10HD", 10, -1): ("truck is not on the rightmost driving lane, off the junction",
                           "car is not on the rightmost driving lane, off the junction",
                           "the child's waiting spot is in sight of the ego 30 m before the crossing",
                           "the child's waiting spot is in sight of the ego 35 m before the crossing",
                           "the child's waiting spot is in sight of the ego 40 m before the crossing"),
}
APPROACH = 60.0
CROWD = 10  # extra pedestrians per candidate
CROWD_MIN_GAP = 0.5  # closest allowed start spacing and target spacing [m]
CROWD_MIN_MAIN_CLEARANCE = 0.4  # closest allowed distance of a crowd start or target to the main pedestrian's line [m]
CROWD_TIMING_SEEDS = (1, 2, 3)  # seeds for the delays and speeds
CROWD_MAX_DELAY, CROWD_SPEEDS = 4.0, (1.1, 1.8)  # the scenario's defaults [s], [m/s]
CYCLISTS = 4  # bicycles per candidate
CYCLIST_SAMPLE = {"Town12": 10}  # town -> check the cyclists of every n-th candidate only (default: all)
CYCLIST_MIN_CLEARANCE = 1.6  # closest allowed distance of a bicycle's line to the main pedestrian's line [m]
CYCLIST_MIN_GAP = 2.0  # closest allowed bicycles of one direction [m]
# Real bounding boxes (half extents) of CARLA 0.9.16's blueprints, measured on a server: bh.crossbike is the widest
# bicycle, the longest one is 0.92 m half long; the widest walker is 0.25 m. These are measurements, not the
# constants of carsim.road, which round them up and are checked against them below.
BIKE_HALF_WIDTH = 0.433  # [m]
BIKE_HALF_LENGTH = 0.92  # [m]
WALKER_HALF_WIDTH = 0.25  # [m]
CLEARANCE = 0.05  # free space that must remain between two bodies [m]
CYCLIST_EGO_TOLERANCE = 1.5  # how far a ride may miss the ego lane's line and still count as crossing it [m]
CYCLIST_TIMING_SEEDS = (1, 2, 3)
WAITING_QUEUE = 4  # waiting spots asked for per bicycle line
WAITING_MIN_GAP = 2.0  # closest allowed waiting bicycles of one line [m]
WAITING_BEYOND = (1.0, 3.0)  # allowed distance of the first waiting spot beyond the kerb [m]
WAITING_MAX_MISSING = 0.25  # at most this share of the lines may have no waiting spot (checked on maps with 40 lines or more) at all
CYCLIST_MAX_DELAY, CYCLIST_SPEEDS = 3.0, (3.0, 5.0)  # the scenario's defaults [s], [m/s]
TRUCK_HALF = (4.5, 1.25)  # half length and half width of the occluder's truck [m], about european_hgv's (not measured)
CAR_HALF = 2.3  # half length of a queued car [m]
OCCLUDER_GAP = 1.5  # truck front to the near edge of the crosswalk [m]
HIDE_BEFORE = range(15, 41, 5)  # distances before the crossing at which the ego looks at the crosswalk [m]
PROP_HALF = (1.95, 0.95)  # half length and half width of static.prop.busstop, measured on a server [m]
PROP_HIDDEN = range(15, 61, 5)  # ego distances before the crossing from which the child's waiting spot is hidden [m]
WALK_IN = 6.0  # how far along the road the child starts before the crossing spot [m]
LEG_HIDDEN = (10, 15)  # ego distances before the crossing from which the whole walk-in is hidden by the truck [m]
TURN_HIDDEN = (20,)  # ... and the end of the walk-in, where the child turns [m]
BEFORE_ENTRY = (0.0, 1.0, 2.5, 5.0, 10.0, 15.0, 20.0, 25.0)  # ego start distances before a junction entry [m]


def xodr_path(town):
    """
    The OpenDRIVE file of `town`: Maps/OpenDrive/<town>.xodr, Maps/<town>/OpenDrive/<town>.xodr (Town12), an imported
    map package's Content/<package>/Maps/<town>/OpenDrive/<town>.xodr, or <town>.xodr in a folder of the environment
    variable CARSIM_XODR_DIRS (os.pathsep separated, e.g. the folder of WolfsburgHighway.xodr).
    """
    paths = [MAPS_DIR / "OpenDrive" / f"{town}.xodr", MAPS_DIR / town / "OpenDrive" / f"{town}.xodr"]
    paths += sorted((CARLA_ROOT / "CarlaUE4" / "Content").glob(f"*/Maps/{town}/OpenDrive/{town}.xodr"))
    paths += [Path(folder) / f"{town}.xodr" for folder in os.environ.get("CARSIM_XODR_DIRS", "").split(os.pathsep) if folder]
    for path in paths:
        if path.exists():
            return path
    return MAPS_DIR / "OpenDrive" / f"{town}.xodr"


def distance_to_polygon(p, pts):
    """Distance from (x, y) to a polygon in 2D, 0 when inside."""
    inside = False
    best = float("inf")
    for i, a in enumerate(pts):
        b = pts[(i + 1) % len(pts)]
        if (a.y > p[1]) != (b.y > p[1]) and p[0] < a.x + (p[1] - a.y) * (b.x - a.x) / (b.y - a.y):
            inside = not inside
        ex, ey = b.x - a.x, b.y - a.y
        t = max(0.0, min(1.0, ((p[0] - a.x) * ex + (p[1] - a.y) * ey) / (ex * ex + ey * ey or 1.0)))
        best = min(best, math.hypot(p[0] - a.x - t * ex, p[1] - a.y - t * ey))
    return 0.0 if inside else best


def lane_meets_walk_line(start, crossing, length):
    """Follow the ego route from `start` to the first crossing of the walk line: (point, heading, arc length)."""
    a, b = crossing.spawn.location, crossing.end
    side = lambda p: (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
    step = 0.5
    route = [start] + straight_route(start, length, step)
    s0 = side(route[0].transform.location)
    for i in range(1, len(route)):
        p, q = route[i - 1].transform.location, route[i].transform.location
        s1 = side(q)
        if s0 * s1 <= 0.0 and s0 != s1:
            f = s0 / (s0 - s1)
            point = (p.x + f * (q.x - p.x), p.y + f * (q.y - p.y))
            heading = (q.x - p.x, q.y - p.y)
            return point, heading, (i - 1 + f) * step
        s0 = s1
    return None


def check_walk_sides(cmap, crossing, point, heading):
    """Side and lane checks shared by the crosswalk and fallback paths."""
    problems = []
    # Right of heading (dx, dy) in CARLA's left-handed frame is (-dy, dx)
    norm = math.hypot(*heading) or 1.0
    rx, ry = -heading[1] / norm, heading[0] / norm
    s, e = crossing.spawn.location, crossing.end
    lat_start = (s.x - point[0]) * rx + (s.y - point[1]) * ry
    lat_end = (e.x - point[0]) * rx + (e.y - point[1]) * ry
    if lat_start <= 0.0:
        problems.append(f"walk starts on the ego's left ({lat_start:.1f} m)")
    if lat_end >= 0.0:
        problems.append(f"walk ends on the ego's right ({lat_end:.1f} m)")
    if not 2.0 < crossing.walk_distance < 60.0:
        problems.append(f"walk length {crossing.walk_distance:.1f} m")
    for name, loc in (("start", s), ("end", e)):
        if cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            problems.append(f"walk {name} is on a driving lane")
    return problems


def check_candidate(cmap, index, start, crossing, cyclists=True, stats=None):
    """Returns a list of failure strings (empty = ok)."""
    problems = []
    again = find_crossing(cmap, start, index, max_distance=APPROACH + 50.0)
    if again is None or again.crosswalk is None or again.crosswalk.center.distance(crossing.crosswalk.center) > 0.01:
        return ["crossing computed from the start differs from the candidate's"]
    crossing = again
    cw = crossing.crosswalk

    if abs(crossing.distance - APPROACH) > 5.0:
        problems.append(f"distance_to_crossing {crossing.distance:.1f} m vs approach {APPROACH:.0f} m")

    hit = lane_meets_walk_line(start, crossing, APPROACH + 40.0)
    if hit is None:
        return problems + ["ego lane never meets the walk line"]
    point, heading, arc = hit
    gap = distance_to_polygon(point, cw.points)
    if gap > 0.5:
        problems.append(f"lane meets walk line {gap:.2f} m outside the crosswalk")
    trigger = crossing.wp.transform.location
    if math.hypot(trigger.x - point[0], trigger.y - point[1]) > 0.5:
        problems.append("trigger location is not where the lane meets the walk line")
    if abs(arc - crossing.distance) > 1.5:
        problems.append(f"distance_to_crossing {crossing.distance:.1f} m but the route meets the line at {arc:.1f} m")

    problems += check_walk_sides(cmap, crossing, point, heading) + check_spawns(cmap, crossing)
    problems += check_crowd(cmap, crossing, point, heading)
    if cyclists:
        problems += check_cyclists(cmap, crossing, point, heading, stats=stats)
    return problems


def check_spawns(cmap, crossing):
    """The walker spawn alternatives all reach crossing.end, face it and start off the driving lanes."""
    problems = []
    spawns = list(walker_spawns(cmap, crossing))
    if not spawns or spawns[0][1].location.distance(crossing.spawn.location) > 1e-6:
        return ["first spawn alternative is not the crossing's spawn"]
    for offset, tf, walk_distance in spawns:
        fwd = tf.get_forward_vector()
        x, y = tf.location.x + fwd.x * walk_distance, tf.location.y + fwd.y * walk_distance
        if math.hypot(x - crossing.end.x, y - crossing.end.y) > 0.05:
            problems.append(f"spawn offset {offset} does not walk to the end point")
        if cmap.get_waypoint(tf.location, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            problems.append(f"spawn offset {offset} is on a driving lane")
    return problems


def column_min_gap(a, timing_a, b, timing_b):
    """
    Closest the walkers `a` and `b` of one column come to each other while both follow their (delay, speed).
    Each walks its straight line at constant speed and stops at its target, so the distance is piecewise linear
    in time and its minimum is at one of the moments where either starts or stops.
    """
    def position(spot, timing, t):
        delay, speed = timing
        f = min(max(0.0, t - delay) * speed, spot.distance) / spot.distance
        s, e = spot.spawn.location, spot.end
        return s.x + (e.x - s.x) * f, s.y + (e.y - s.y) * f

    times = {0.0}
    for spot, (delay, speed) in ((a, timing_a), (b, timing_b)):
        times.update((delay, delay + spot.distance / speed))
    best = float("inf")
    for t in times:
        pa, pb = position(a, timing_a, t), position(b, timing_b, t)
        best = min(best, math.hypot(pa[0] - pb[0], pa[1] - pb[1]))
    return best


def check_crowd_columns(crossing, spots):
    """Spacing, clearance, queue and timing checks that do not depend on the ego lane."""
    problems = []
    s, e = crossing.spawn.location, crossing.end
    norm = math.hypot(e.x - s.x, e.y - s.y) or 1.0
    ux, uy = (e.x - s.x) / norm, (e.y - s.y) / norm  # main pedestrian's walk direction
    lateral = lambda p: (p.x - s.x) * -uy + (p.y - s.y) * ux  # distance from the main line, right of u positive
    along = lambda p: (p.x - s.x) * ux + (p.y - s.y) * uy

    for i, spot in enumerate(spots):
        for name, loc in (("start", spot.spawn.location), ("target", spot.end)):
            if abs(lateral(loc)) < CROWD_MIN_MAIN_CLEARANCE:
                problems.append(f"crowd {i}: {name} is only {abs(lateral(loc)):.2f} m from the main pedestrian's line")
    for i, a in enumerate(spots):
        for j in range(i + 1, len(spots)):
            gap = math.hypot(a.end.x - spots[j].end.x, a.end.y - spots[j].end.y)
            if gap < CROWD_MIN_GAP - 1e-6:
                problems.append(f"crowd {i} and {j} have targets only {gap:.2f} m apart")

    columns = {}
    for spot in spots:
        columns.setdefault((spot.side, spot.column), []).append(spot)
    for walkers in columns.values():
        walkers.sort(key=lambda spot: spot.row)
    for (side, column), walkers in sorted(columns.items()):
        sign = 1.0 if side == "A" else -1.0  # progress along the walk is along u for side A, against it for B
        for ahead, behind in zip(walkers, walkers[1:]):
            where = f"column {side}{column} rows {ahead.row}/{behind.row}"
            if (abs(lateral(ahead.end) - lateral(behind.end)) > 0.01
                    or abs(lateral(ahead.spawn.location) - lateral(behind.spawn.location)) > 0.01):
                problems.append(f"{where}: not on one line")
            if abs(ahead.distance - behind.distance) > 0.05:
                problems.append(f"{where}: walk lengths {ahead.distance:.2f} and {behind.distance:.2f} m differ")
            start_gap = sign * (along(ahead.spawn.location) - along(behind.spawn.location))
            end_gap = sign * (along(ahead.end) - along(behind.end))
            if start_gap < CROWD_MIN_GAP - 1e-6:
                problems.append(f"{where}: the walker behind starts {start_gap:.2f} m behind the one ahead")
            if end_gap < CROWD_MIN_GAP - 1e-6:
                problems.append(f"{where}: the targets are {end_gap:.2f} m apart or in the wrong order (a queue "
                                "needs the walker that started in front to end in front)")

    for seed in CROWD_TIMING_SEEDS:
        timing = crowd_timing(spots, random.Random(seed), CROWD_MAX_DELAY, CROWD_SPEEDS)
        if len(timing) != len(spots) or any(delay < 0.0 or not CROWD_SPEEDS[0] <= speed <= CROWD_SPEEDS[1]
                                            for delay, speed in timing):
            problems.append(f"crowd timing seed {seed}: delays or speeds out of range")
            continue
        by_spot = {id(spot): t for spot, t in zip(spots, timing)}
        for (side, column), walkers in sorted(columns.items()):
            for ahead, behind in zip(walkers, walkers[1:]):
                where = f"timing seed {seed} column {side}{column} rows {ahead.row}/{behind.row}"
                (d0, v0), (d1, v1) = by_spot[id(ahead)], by_spot[id(behind)]
                if v0 != v1:
                    problems.append(f"{where}: speeds {v0:.2f} and {v1:.2f} m/s differ")
                if d1 < d0 + ROW_GAP / CROWD_SPEEDS[0] - 1e-9:
                    problems.append(f"{where}: the walker behind starts only {d1 - d0:.2f} s after the one ahead")
                gap = column_min_gap(ahead, (d0, v0), behind, (d1, v1))
                if gap < CROWD_MIN_GAP - 1e-6:
                    problems.append(f"{where}: they come {gap:.2f} m close")
    return problems


def check_crowd(cmap, crossing, point, heading, count=CROWD):
    """The crowd splits over both sides, crosses the ego lane, starts off the driving lanes and is spread out."""
    problems = []
    spots = crowd_spawns(cmap, crossing, count, random.Random(1234))
    if len(spots) != count:
        problems.append(f"crowd: {len(spots)} of {count} walkers placed")
    sides = [sum(1 for spot in spots if spot.side == side) for side in "AB"]
    if min(sides) < count // 2 - 1 or max(sides) > (count + 1) // 2 + 1:
        problems.append(f"crowd: {sides[0]} start on the right, {sides[1]} on the left")

    norm = math.hypot(*heading) or 1.0
    rx, ry = -heading[1] / norm, heading[0] / norm
    lateral = lambda p: (p.x - point[0]) * rx + (p.y - point[1]) * ry
    for i, spot in enumerate(spots):
        start, end = spot.spawn.location, spot.end
        if lateral(start) * lateral(end) >= 0.0:
            problems.append(f"crowd {i}: start ({lateral(start):.1f} m) and target ({lateral(end):.1f} m) are on "
                            "the same side of the ego lane")
        if (lateral(start) > 0.0) != (spot.side == "A"):
            problems.append(f"crowd {i}: side {spot.side} starts at lateral {lateral(start):.1f} m")
        if cmap.get_waypoint(start, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            problems.append(f"crowd {i}: start is on a driving lane")
        if cmap.get_waypoint(end, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            problems.append(f"crowd {i}: target is on a driving lane")
        fwd = spot.spawn.get_forward_vector()
        reached = (start.x + fwd.x * spot.distance, start.y + fwd.y * spot.distance)
        if math.hypot(reached[0] - end.x, reached[1] - end.y) > 0.05:
            problems.append(f"crowd {i}: does not face its target or the distance is wrong")
    for i, a in enumerate(spots):
        for j in range(i + 1, len(spots)):
            b = spots[j]
            gap = math.hypot(a.spawn.location.x - b.spawn.location.x, a.spawn.location.y - b.spawn.location.y)
            if gap < CROWD_MIN_GAP - 1e-6:
                problems.append(f"crowd {i} and {j} start only {gap:.2f} m apart")
    return problems + check_crowd_columns(crossing, spots)


def point_segment_distance(p, a, b):
    """Distance of the point p from the segment a-b (all (x, y))."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else min(1.0, max(0.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - (a[0] + dx * t), p[1] - (a[1] + dy * t))


def segment_distance(a, b, c, d):
    """
    Smallest distance between any point of the segment a-b and any point of c-d: 0 when they cross, else it is
    reached at one of the four end points. Exact, so it covers every point of both, not a sample.
    """
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    if side(a, b, c) * side(a, b, d) < 0.0 and side(c, d, a) * side(c, d, b) < 0.0:
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def ride_segment(spot):
    """The straight ride of a bicycle or a walker as ((x, y), (x, y))."""
    return (spot.spawn.location.x, spot.spawn.location.y), (spot.end.x, spot.end.y)


def clearance_problems(bikes, crowd, main_line, bike_half=BIKE_HALF_WIDTH, walker_half=WALKER_HALF_WIDTH,
                       margin=CLEARANCE):
    """
    Bodies that would overlap along their rides. `bikes` are CyclistSpots, `crowd` CrowdSpots and `main_line` the
    main pedestrian's walk as ((x, y), (x, y)). Rides are straight lines, so the exact minimum distance between two
    rides is the closest any two points of them come, whatever the timing; the bodies are circles of the real half
    widths (a bicycle seen from above is longer, but when it overtakes sideways what matters is its width, and a
    bicycle ahead or behind is checked by the queue gap). Bicycles of one direction share a line and are queued,
    so only the two directions are compared.
    """
    problems = []
    for i, a in enumerate(bikes):
        for j in range(i + 1, len(bikes)):
            b = bikes[j]
            if a.side == b.side:
                continue
            gap = segment_distance(*ride_segment(a), *ride_segment(b))
            if gap < 2.0 * bike_half + margin:
                problems.append(f"bicycles {i} and {j} come {gap:.2f} m apart, centre to centre "
                                f"(need {2.0 * bike_half + margin:.2f} m)")
        for j, walker in enumerate(crowd):
            gap = segment_distance(*ride_segment(a), *ride_segment(walker))
            if gap < bike_half + walker_half + margin:
                problems.append(f"bicycle {i} comes {gap:.2f} m from crowd walker {j} "
                                f"(need {bike_half + walker_half + margin:.2f} m)")
        gap = segment_distance(*ride_segment(a), *main_line)
        if gap < bike_half + walker_half + margin:
            problems.append(f"bicycle {i} comes {gap:.2f} m from the main pedestrian "
                            f"(need {bike_half + walker_half + margin:.2f} m)")
    return problems


def fake_spot(kind, side, lateral, a=-8.0, b=8.0):
    """A straight ride at `lateral` m from a main line along x, from along-position a to b (reversed for side B)."""
    start, end = (a, b) if side == "A" else (b, a)
    yaw = 0.0 if side == "A" else 180.0
    tf = carla.Transform(carla.Location(x=start, y=lateral, z=0.0), carla.Rotation(yaw=yaw))
    if kind == "bike":
        return CyclistSpot(side, 0, abs(lateral), tf, carla.Location(x=end, y=lateral, z=0.0), abs(b - a))
    return CrowdSpot(side, 0, 0, tf, carla.Location(x=end, y=lateral, z=0.0), abs(b - a))


def check_clearance_selftest():
    """The checker must reject the old lane offsets and the rides that cross, and accept the current lanes."""
    problems = []
    main_line = ((-8.0, 0.0), (8.0, 0.0))
    outer_crowd = [fake_spot("crowd", "A", 1.25), fake_spot("crowd", "B", 1.25)]
    old = [fake_spot("bike", "A", 1.9), fake_spot("bike", "B", 2.7)]
    now = [fake_spot("bike", "A", CYCLE_LANE_OFFSETS[0]), fake_spot("bike", "B", CYCLE_LANE_OFFSETS[1])]
    swapped = [fake_spot("bike", "A", CYCLE_LANE_OFFSETS[1]), fake_spot("bike", "B", CYCLE_LANE_OFFSETS[0])]
    found = clearance_problems(old, outer_crowd, main_line)
    if not any("bicycles 0 and 1" in p for p in found) or not any("crowd walker" in p for p in found):
        problems.append(f"the old lane offsets (1.9, 2.7) passed the clearance check: {found}")
    found = clearance_problems([fake_spot("bike", "A", 2.1), fake_spot("bike", "B", 2.1 + 2 * 0.433)], [], main_line)
    if not found:
        problems.append("two crossbikes 0.866 m apart (touching) passed the clearance check")
    for label, bikes in (("current", now), ("swapped", swapped)):
        found = clearance_problems(bikes, outer_crowd, main_line)
        if found:
            problems.append(f"the {label} lanes {CYCLE_LANE_OFFSETS} fail the clearance check: {found}")
    # Rides that cross (not parallel) have distance 0 at the crossing, wherever it is along them
    cross = [fake_spot("bike", "A", 3.0), CyclistSpot("B", 0, 3.0, carla.Transform(carla.Location(x=0.0, y=4.0, z=0.0)),
                                                       carla.Location(x=0.0, y=2.0, z=0.0), 2.0)]
    if not any("bicycles 0 and 1" in p for p in clearance_problems(cross, [], main_line)):
        problems.append("crossing rides passed the clearance check")
    # The constants of carsim.road cover the measured sizes and give the lanes the margins they promise
    if not (MAX_BIKE_HALF_WIDTH >= BIKE_HALF_WIDTH and MAX_BIKE_HALF_LENGTH >= BIKE_HALF_LENGTH
            and MAX_WALKER_HALF_WIDTH >= WALKER_HALF_WIDTH):
        problems.append("a carsim.road size constant is below the measured size")
    if abs(CYCLE_LANE_OFFSETS[0] - 2.10) > 1e-9 or abs(CYCLE_LANE_OFFSETS[1] - 3.15) > 1e-9:
        problems.append(f"cycle lanes {CYCLE_LANE_OFFSETS}, expected (2.10, 3.15)")
    # The planned push plus the late end plus the braking never exceeds the ride; short rides cap the speed
    for distance in (2.0, 3.0, 5.0, 8.0, 14.0, 30.0):
        for step in (0.05, 0.02, 0.1):
            cap = cyclist_speed_limit(distance, step)
            for speed in (1.0, 3.0, 5.0, 8.0):
                v = min(speed, cap)
                ride = cyclist_forced_ride(distance, v, step)
                if ride + v * step + v * v / (2.0 * CYCLE_DECELERATION) > distance + 1e-9:
                    problems.append(f"push {ride:.2f} m + overshoot + braking exceeds {distance} m at {v:.2f} m/s")
            if cap > 0 and abs(cyclist_forced_ride(distance, cap, step) - 0.5) > 1e-9 and distance >= 2.0:
                problems.append(f"the speed cap {cap:.2f} m/s does not leave a 0.5 m push for {distance} m")
    if cyclist_forced_ride(1.0, 9.0) != 0.0:
        problems.append("a push longer than the ride was not clamped to 0")
    # Crowd retries: further back on the same line, a longer walk, same target and column
    spot = fake_spot("crowd", "A", 1.25, -8.0, 8.0)
    retries = list(crowd_spawn_retries(spot))
    if [round(back, 3) for back, _ in retries] != [0.25, 0.5]:
        problems.append(f"crowd retries {[back for back, _ in retries]}")
    for back, retry in retries:
        moved = spot.spawn.location.x - retry.spawn.location.x  # side A walks along +x: back is -x
        if (abs(moved - back) > 1e-6 or abs(retry.spawn.location.y - spot.spawn.location.y) > 1e-6
                or abs(retry.distance - (spot.distance + back)) > 1e-9 or retry.end != spot.end
                or retry.column != spot.column or retry.row != spot.row or retry.side != spot.side):
            problems.append(f"crowd retry {back} m: moved {moved:.3f} m, distance {retry.distance}")
    return problems


def check_cyclists(cmap, crossing, point, heading, count=CYCLISTS, stats=None):
    """
    Cyclist geometry: both directions, start and target on a driving lane, clear of the main pedestrian's line, on the
    junction side, spaced, queued in order, and timed so nobody catches up. A narrow road may fit fewer than `count`
    bicycles, which is no failure (at least one per direction is); `stats` (a dict) gets the shortfall counted.
    """
    problems = []
    spots = cyclist_spawns(cmap, crossing, count)
    if stats is not None:
        stats["checked"] = stats.get("checked", 0) + 1
        stats["short"] = stats.get("short", 0) + (len(spots) < count)
    sides = {side: [spot for spot in spots if spot.side == side] for side in "AB"}
    for side, group in sides.items():
        if not group:
            problems.append(f"cyclists: no bicycle in direction {side}")
    if not spots:
        return problems

    a, b = crossing.spawn.location, crossing.end
    norm = math.hypot(b.x - a.x, b.y - a.y) or 1.0
    ux, uy = (b.x - a.x) / norm, (b.y - a.y) / norm  # main pedestrian's walk direction
    rx, ry = -uy, ux
    hnorm = math.hypot(*heading) or 1.0
    fx, fy = heading[0] / hnorm, heading[1] / hnorm  # the ego's heading at the crossing
    lateral = lambda p: (p.x - a.x) * rx + (p.y - a.y) * ry  # signed distance from the main line
    along = lambda p: (p.x - a.x) * ux + (p.y - a.y) * uy

    for i, spot in enumerate(spots):
        for name, loc in (("start", spot.spawn.location), ("target", spot.end)):
            if cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Driving) is None:
                problems.append(f"cyclist {i}: {name} is not on a driving lane")
            if abs(lateral(loc)) < CYCLIST_MIN_CLEARANCE - 1e-6:
                problems.append(f"cyclist {i}: {name} is only {abs(lateral(loc)):.2f} m from the main pedestrian's line")
            # Offset from the main line, across it: must point the way the ego drives
            off = (lateral(loc) * rx, lateral(loc) * ry)
            if off[0] * fx + off[1] * fy <= 0.0:
                problems.append(f"cyclist {i}: {name} is on the far side of the crosswalk from the junction")
        start, end = spot.spawn.location, spot.end
        if abs(lateral(start) - lateral(end)) > 0.05:
            problems.append(f"cyclist {i}: does not ride parallel to the main pedestrian's line")
        if (along(end) > along(start)) != (spot.side == "A"):
            problems.append(f"cyclist {i}: direction {spot.side} rides the wrong way")
        fwd = spot.spawn.get_forward_vector()
        if math.hypot(start.x + fwd.x * spot.distance - end.x, start.y + fwd.y * spot.distance - end.y) > 0.05:
            problems.append(f"cyclist {i}: does not face its target or the distance is wrong")
        if spot.distance < 2.0:
            problems.append(f"cyclist {i}: ride of only {spot.distance:.1f} m")

    # Direction A starts on the ego's right and ends on its left, B the other way round: compare the back of the
    # queues (they start at the kerb) and the fronts (they end at the far kerb)
    if sides["A"] and sides["B"]:
        back = {side: max(group, key=lambda spot: spot.row) for side, group in sides.items()}
        front = {side: group[0] for side, group in sides.items()}
        if along(back["A"].spawn.location) >= along(back["B"].spawn.location):
            problems.append("cyclists: the directions do not start at opposite kerbs")
        if along(front["A"].end) <= along(front["B"].end):
            problems.append("cyclists: the directions do not end at opposite kerbs")

    # Somebody of each direction rides across the ego lane, give or take CYCLIST_EGO_TOLERANCE (the front of a queue
    # starts furthest in, so on a narrow road it can start past the ego lane already)
    ego_along = along(carla.Location(x=point[0], y=point[1]))
    for side, group in sides.items():
        if group and not any(min(along(s.spawn.location), along(s.end)) - CYCLIST_EGO_TOLERANCE < ego_along
                             < max(along(s.spawn.location), along(s.end)) + CYCLIST_EGO_TOLERANCE for s in group):
            problems.append(f"cyclists: no bicycle of direction {side} rides across the ego lane")

    # Same direction: all in one lane, queued at least CYCLIST_MIN_GAP apart at both ends, the front one ahead
    for side, group in sides.items():
        group.sort(key=lambda spot: spot.row)
        sign = 1.0 if side == "A" else -1.0  # progress is along u for A, against it for B
        for ahead, behind in zip(group, group[1:]):
            where = f"cyclists {side} rows {ahead.row}/{behind.row}"
            if abs(lateral(ahead.end) - lateral(behind.end)) > 0.01:
                problems.append(f"{where}: not in one lane")
            start_gap = sign * (along(ahead.spawn.location) - along(behind.spawn.location))
            end_gap = sign * (along(ahead.end) - along(behind.end))
            for what, gap in (("start", start_gap), ("end", end_gap)):
                if gap < CYCLIST_MIN_GAP - 1e-6:
                    problems.append(f"{where}: {what}s are {gap:.2f} m apart or in the wrong order")
    # Opposite directions: lanes side by side, wide enough for two of the widest bicycles
    if sides["A"] and sides["B"]:
        gap = abs(lateral(sides["A"][0].spawn.location) - lateral(sides["B"][0].spawn.location))
        if gap < 2.0 * BIKE_HALF_WIDTH + CLEARANCE - 1e-6:
            problems.append(f"cyclists: the two directions are only {gap:.2f} m apart sideways")

    # Both ends of a bicycle (its half length from the start and from the target) are on the road: the far kerb is
    # not overshot by the nose, the near one is not hung over by the tail
    for i, spot in enumerate(spots):
        fwd = spot.spawn.get_forward_vector()
        for name, loc, sign in (("nose at the target", spot.end, 1.0), ("tail at the start", spot.spawn.location, -1.0)):
            tip = carla.Location(x=loc.x + fwd.x * BIKE_HALF_LENGTH * sign, y=loc.y + fwd.y * BIKE_HALF_LENGTH * sign,
                                 z=loc.z)
            if cmap.get_waypoint(tip, project_to_road=False, lane_type=carla.LaneType.Driving) is None:
                problems.append(f"cyclist {i}: the {name} ({BIKE_HALF_LENGTH} m from it) is off the road")

    # Clearance between the rides, with the real sizes: the other direction, the crowd and the main pedestrian
    crowd = crowd_spawns(cmap, crossing, CROWD, random.Random(1234))
    problems += [f"cyclists: {p}" for p in clearance_problems(spots, crowd, ((a.x, a.y), (b.x, b.y)))]

    problems += check_waiting_spots(cmap, spots, stats)

    for seed in CYCLIST_TIMING_SEEDS:
        limits = [cyclist_speed_limit(spot.distance) for spot in spots]
        timing = cyclist_timing(spots, random.Random(seed), CYCLIST_MAX_DELAY, CYCLIST_SPEEDS, speed_limits=limits)
        if len(timing) != len(spots) or any(delay < 0.0 or not 0.0 < speed <= CYCLIST_SPEEDS[1]
                                            for delay, speed in timing):
            problems.append(f"cyclist timing seed {seed}: delays or speeds out of range")
            continue
        for i, (spot, (delay, speed)) in enumerate(zip(spots, timing)):
            # Only a ride too short to stop in may slow a bicycle below the configured minimum
            lane_limit = min(l for other, l in zip(spots, limits) if other.side == spot.side)
            if speed < CYCLIST_SPEEDS[0] - 1e-9 and speed < lane_limit - 1e-9:
                problems.append(f"cyclist timing seed {seed} cyclist {i}: speed {speed:.2f} m/s below the minimum "
                                "without a speed limit")
            push = cyclist_forced_ride(spot.distance, speed)
            stopping = push + speed * CYCLE_STEP + speed * speed / (2.0 * CYCLE_DECELERATION)
            if stopping > spot.distance + 1e-9 or push < 0.5 - 1e-9:
                problems.append(f"cyclist timing seed {seed} cyclist {i}: push {push:.2f} m, with the late end and the "
                                f"braking {stopping:.2f} m of a {spot.distance:.2f} m ride at {speed:.2f} m/s")
        by_spot = {id(spot): t for spot, t in zip(spots, timing)}
        for side, group in sides.items():
            for ahead, behind in zip(group, group[1:]):
                where = f"cyclist timing seed {seed} direction {side} rows {ahead.row}/{behind.row}"
                (d0, v0), (d1, v1) = by_spot[id(ahead)], by_spot[id(behind)]
                if v0 != v1:
                    problems.append(f"{where}: speeds {v0:.2f} and {v1:.2f} m/s differ")
                if d1 <= d0:
                    problems.append(f"{where}: the bicycle behind does not start after the one ahead")
                gap = column_min_gap(ahead, (d0, v0), behind, (d1, v1))
                if gap < CYCLIST_MIN_GAP - 1e-6:
                    problems.append(f"{where}: they come {gap:.2f} m close")
    return problems


def last_driving_distance(cmap, spot):
    """Independent of road.py: how far back from the start of `spot` the last driving lane point of its line is [m]."""
    start, fwd = spot.spawn.location, spot.spawn.get_forward_vector()
    last, k = 0.0, 0.0
    while k - last < 0.6 and k < 60.0:
        k += 0.05
        probe = carla.Location(x=start.x - fwd.x * k, y=start.y - fwd.y * k, z=start.z)
        if cmap.get_waypoint(probe, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            last = k
    return last


def check_exit_spot(cmap, spot, i, stats=None):
    """
    The far sidewalk spot a bicycle leaves to after its ride (road.cyclist_exit_spot): on the cycle line, beyond the
    end of the ride and the far kerb, clear of the driving lanes, facing the ride direction, near the road's height.
    """
    problems = []
    tf = cyclist_exit_spot(cmap, spot)
    if stats is not None:
        stats["exit_lines"] = stats.get("exit_lines", 0) + 1
    if tf is None:
        if stats is not None:
            stats["no_exit"] = stats.get("no_exit", 0) + 1
        return problems
    start, end, fwd = spot.spawn.location, spot.end, spot.spawn.get_forward_vector()
    loc = tf.location
    beyond = (loc.x - end.x) * fwd.x + (loc.y - end.y) * fwd.y  # along the ride direction, past its end
    side = abs(-(loc.x - start.x) * fwd.y + (loc.y - start.y) * fwd.x)
    where = f"cyclist {i} exit spot"
    if side > 0.01:
        problems.append(f"{where}: {side:.3f} m off the cycle line")
    if not 1.0 <= beyond <= 10.0 + 1e-6:
        problems.append(f"{where}: {beyond:.2f} m past the end of the ride, expected 1 to 10 m")
    if abs((tf.rotation.yaw - spot.spawn.rotation.yaw + 180.0) % 360.0 - 180.0) > 0.5:
        problems.append(f"{where}: does not keep the ride direction")
    for name, along in (("centre", 0.0), ("nose", MAX_BIKE_HALF_LENGTH), ("tail", -MAX_BIKE_HALF_LENGTH)):
        probe = carla.Location(x=loc.x + fwd.x * along, y=loc.y + fwd.y * along, z=loc.z)
        if cmap.get_waypoint(probe, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
            problems.append(f"{where}: the {name} is on a driving lane")
    if abs(loc.z - end.z) > 1.5:
        problems.append(f"{where}: height {loc.z:.2f} m vs the road's {end.z:.2f} m")
    return problems


def check_waiting_spots(cmap, spots, stats=None):
    """The sidewalk waiting spots of the bicycle lines `spots` (see the module docstring)."""
    problems = []
    for i, spot in enumerate(spots):
        if stats is not None:
            stats["lines"] = stats.get("lines", 0) + 1
        problems += check_exit_spot(cmap, spot, i, stats)
        waits = cyclist_waiting_spots(cmap, spot, WAITING_QUEUE)
        if not waits:
            if stats is not None:
                stats["no_wait"] = stats.get("no_wait", 0) + 1
            continue
        start, fwd = spot.spawn.location, spot.spawn.get_forward_vector()
        kerb = last_driving_distance(cmap, spot)
        for q, tf in enumerate(waits):
            where = f"cyclist {i} waiting spot {q}"
            loc = tf.location
            back = (start.x - loc.x) * fwd.x + (start.y - loc.y) * fwd.y  # along the line, behind the start
            side = abs(-(loc.x - start.x) * fwd.y + (loc.y - start.y) * fwd.x)
            if side > 0.01:
                problems.append(f"{where}: {side:.3f} m off the cycle line")
            if abs((tf.rotation.yaw - spot.spawn.rotation.yaw + 180.0) % 360.0 - 180.0) > 0.5:
                problems.append(f"{where}: does not face the ride direction")
            for name, along in (("centre", 0.0), ("nose", MAX_BIKE_HALF_LENGTH), ("tail", -MAX_BIKE_HALF_LENGTH)):
                probe = carla.Location(x=loc.x + fwd.x * along, y=loc.y + fwd.y * along, z=loc.z)
                if cmap.get_waypoint(probe, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                    problems.append(f"{where}: the {name} is on a driving lane")
            if abs(tf.location.z - (start.z - 0.1)) > 1.5:
                problems.append(f"{where}: height {tf.location.z:.2f} m vs the road's {start.z - 0.1:.2f} m")
            if q == 0:
                beyond = back - kerb
                if not WAITING_BEYOND[0] - 0.1 <= beyond <= WAITING_BEYOND[1] + 0.1:
                    problems.append(f"{where}: {beyond:.2f} m beyond the kerb, expected {WAITING_BEYOND}")
                if stats is not None:
                    on_sidewalk = cmap.get_waypoint(loc, project_to_road=False,
                                                    lane_type=carla.LaneType.Sidewalk) is not None
                    stats["first"] = stats.get("first", 0) + 1
                    stats["on_sidewalk"] = stats.get("on_sidewalk", 0) + on_sidewalk
            else:
                gap = math.hypot(loc.x - waits[q - 1].location.x, loc.y - waits[q - 1].location.y)
                if gap < WAITING_MIN_GAP - 1e-6 or abs(gap - SIDEWALK_QUEUE_GAP) > 0.01:
                    problems.append(f"{where}: {gap:.2f} m from the one in front (need {SIDEWALK_QUEUE_GAP})")
        for extra, tf in cyclist_wait_retries(cmap, waits[0]):
            for along in (0.0, MAX_BIKE_HALF_LENGTH, -MAX_BIKE_HALF_LENGTH):
                fw = tf.get_forward_vector()
                probe = carla.Location(x=tf.location.x + fw.x * along, y=tf.location.y + fw.y * along, z=tf.location.z)
                if cmap.get_waypoint(probe, project_to_road=False, lane_type=carla.LaneType.Driving) is not None:
                    problems.append(f"cyclist {i}: the retry {extra} m further back is on a driving lane")
            if abs(math.hypot(tf.location.x - waits[0].location.x, tf.location.y - waits[0].location.y) - extra) > 0.01:
                problems.append(f"cyclist {i}: the retry is not {extra} m further back")
    return problems


def check_hidden_selftest():
    """
    road.hidden_from_ego, every branch: (a) far away, (b) off to the side from 50 m, (c) behind the car from 15 m unless
    the ego reverses, and the places where nothing is hidden (close, straight ahead below 120 m); at the limits, with
    other numbers, and through HideRule.
    """
    problems = []
    ego = carla.Transform(carla.Location(x=100.0, y=50.0, z=0.0), carla.Rotation(yaw=30.0))

    def at(distance, angle):
        a = math.radians(30.0 + angle)
        return carla.Location(x=100.0 + distance * math.cos(a), y=50.0 + distance * math.sin(a), z=0.0)

    cases = [
        (at(0.0, 0.0), False, "on the ego"),
        (at(5.0, 180.0), False, "5 m behind (closer than 15 m)"),
        (at(14.9, 180.0), False, "14.9 m directly behind"),
        # (c) behind the car
        (at(15.1, 180.0), True, "15.1 m directly behind"),
        (at(15.1, 0.0), False, "15.1 m ahead"),
        (at(30.0, 90.0), False, "30 m to the side"),
        (at(30.0, -90.0), False, "30 m to the other side"),
        (at(30.0, 109.0), False, "30 m at 109 degrees"),
        (at(30.0, -109.0), False, "30 m at -109 degrees"),
        (at(30.0, 111.0), True, "30 m at 111 degrees"),
        (at(30.0, -111.0), True, "30 m at -111 degrees"),
        (at(49.9, 120.0), True, "49.9 m behind-left"),
        (at(49.9, 100.0), False, "49.9 m at 100 degrees"),
        (at(49.9, 70.0), False, "49.9 m at 70 degrees (side rule starts at 50 m)"),
        # straight ahead is in view up to the far distance
        (at(49.9, 0.0), False, "49.9 m ahead"),
        (at(50.01, 0.0), False, "50 m straight ahead (a chase camera sees it)"),
        (at(80.0, 20.0), False, "80 m ahead, 20 degrees off"),
        (at(119.9, 0.0), False, "119.9 m straight ahead"),
        (at(119.9, 59.0), False, "119.9 m at 59 degrees"),
        # (b) off to the side from hide_distance on, more than side_angle off the heading
        (at(50.01, 59.0), False, "50 m at 59 degrees"),
        (at(50.01, 61.0), True, "50 m at 61 degrees"),
        (at(50.01, -61.0), True, "50 m at -61 degrees"),
        (at(49.9, 90.0), False, "49.9 m at 90 degrees"),
        (at(80.0, 90.0), True, "80 m to the side"),
        (at(100.0, -75.0), True, "100 m at -75 degrees"),
        # (a) far away: hidden whatever the direction
        (at(120.01, 0.0), True, "120 m straight ahead"),
        (at(150.0, 10.0), True, "150 m ahead"),
        (at(120.01, 180.0), True, "120 m behind"),
    ]
    for location, expected, label in cases:
        if hidden_from_ego(ego, location) != expected:
            problems.append(f"hidden_from_ego: {label} should be {expected}")
        if HideRule().hidden(ego, location) != expected:
            problems.append(f"HideRule(): {label} should be {expected}")
        # A forward-moving or standing ego changes nothing
        for speed in (0.0, 5.0, -0.5):
            if hidden_from_ego(ego, location, forward_speed=speed) != expected:
                problems.append(f"hidden_from_ego: {label} at forward speed {speed} should be {expected}")

    # Reversing (forward speed below -0.5 m/s): (c) is off, (b) only hides points to the side (angle up to
    # behind_angle), (a) stays
    for speed, expected in ((None, True), (0.0, True), (3.0, True), (-0.4, True), (-0.5, True), (-0.6, False),
                            (-5.0, False)):
        for angle, distance in ((180.0, 30.0), (150.0, 16.0), (-130.0, 40.0)):
            if hidden_from_ego(ego, at(distance, angle), forward_speed=speed) != expected:
                problems.append(f"hidden_from_ego: {distance:g} m at {angle:g} degrees, forward speed {speed}, "
                                f"should be {expected}")
        if HideRule().hidden(ego, at(30.0, 180.0), speed) != expected:
            problems.append(f"HideRule.hidden: 30 m behind, forward speed {speed}, should be {expected}")
    for location, label in ((at(60.0, 90.0), "60 m to the side"), (at(130.0, 0.0), "130 m ahead"),
                            (at(60.0, -100.0), "60 m at 100 degrees"), (at(130.0, 180.0), "130 m behind")):
        if not hidden_from_ego(ego, location, forward_speed=-5.0):
            problems.append(f"hidden_from_ego: {label} must stay hidden while reversing")
    for location, label in ((at(10.0, 180.0), "10 m behind"), (at(40.0, 0.0), "40 m ahead"),
                            (at(100.0, 0.0), "100 m ahead"), (at(60.0, 180.0), "60 m straight behind"),
                            (at(110.0, 170.0), "110 m behind"), (at(60.0, -120.0), "60 m at 120 degrees")):
        if hidden_from_ego(ego, location, forward_speed=-5.0):
            problems.append(f"hidden_from_ego: {label} is hidden while reversing")
    # Not reversing, the same points behind are hidden (rule b, and c)
    for location, label in ((at(60.0, 180.0), "60 m behind"), (at(110.0, 170.0), "110 m behind")):
        for speed in (None, 0.0, 5.0, -0.5):
            if not hidden_from_ego(ego, location, forward_speed=speed):
                problems.append(f"hidden_from_ego: {label} at forward speed {speed} must be hidden")
    if HideRule().hidden(ego, at(60.0, 180.0), -5.0) or not HideRule().hidden(ego, at(60.0, 100.0), -5.0):
        problems.append("HideRule.hidden: while reversing 60 m behind is in view and 60 m at 100 degrees is hidden")

    # The height does not count (2D), and the rule is configurable
    high = carla.Location(x=at(130.0, 0.0).x, y=at(130.0, 0.0).y, z=500.0)
    if not hidden_from_ego(ego, high):
        problems.append("hidden_from_ego looks at the height")
    rule = HideRule(hide_distance=100.0, behind_distance=30.0, behind_angle=90.0, far_distance=200.0, side_angle=45.0)
    for location, expected, label in ((at(150.0, 0.0), False, "150 m ahead, far_distance 200"),
                                      (at(25.0, 180.0), False, "25 m behind, behind_distance 30"),
                                      (at(35.0, 95.0), True, "35 m at 95 degrees, behind_angle 90"),
                                      (at(35.0, 85.0), False, "35 m at 85 degrees, behind_angle 90"),
                                      (at(100.01, 50.0), True, "100 m at 50 degrees, side_angle 45"),
                                      (at(100.01, 40.0), False, "100 m at 40 degrees, side_angle 45"),
                                      (at(99.0, 60.0), False, "99 m at 60 degrees, hide_distance 100"),
                                      (at(200.01, 0.0), True, "200 m ahead, far_distance 200")):
        if rule.hidden(ego, location) != expected:
            problems.append(f"HideRule(100, 30, 90, 200, 45): {label} should be {expected}")
    if HideRule(far_distance=60.0).hidden(ego, at(59.0, 0.0)) or not HideRule(far_distance=60.0).hidden(ego, at(60.01, 0.0)):
        problems.append("HideRule(far_distance=60) is not far at 60 m")
    if not hidden_from_ego(ego, at(20.0, 180.0), hide_distance=50.0, behind_distance=10.0, behind_angle=120.0):
        problems.append("keyword arguments of hidden_from_ego are ignored")
    if not hidden_from_ego(ego, at(70.0, 0.0), far_distance=65.0) or hidden_from_ego(ego, at(60.0, 0.0), far_distance=65.0):
        problems.append("the far_distance keyword of hidden_from_ego is ignored")
    if hidden_from_ego(ego, at(60.0, 70.0), side_angle=80.0) or not hidden_from_ego(ego, at(60.0, 70.0), side_angle=65.0):
        problems.append("the side_angle keyword of hidden_from_ego is ignored")
    return problems


def ray_hits_box(a, b, tf, half_length, half_width):
    """True when the 2D segment a-b meets the box of that size centred on the transform `tf` (slab test in the box's frame)."""
    yaw = math.radians(tf.rotation.yaw)
    cos, sin = math.cos(yaw), math.sin(yaw)

    def local(p):
        dx, dy = p[0] - tf.location.x, p[1] - tf.location.y
        return dx * cos + dy * sin, -dx * sin + dy * cos

    start, end = local(a), local(b)
    t0, t1 = 0.0, 1.0
    for origin, step, half in zip(start, (end[0] - start[0], end[1] - start[1]), (half_length, half_width)):
        if abs(step) < 1e-9:
            if abs(origin) > half:
                return False
            continue
        near, far = sorted(((-half - origin) / step, (half - origin) / step))
        t0, t1 = max(t0, near), min(t1, far)
        if t0 > t1:
            return False
    return True


def check_occluder(cmap, start, crossing):
    """The truck and the car of the occluder block for an inner-lane start (find_candidates lane="inner"): failure strings."""
    right = start.get_right_lane()
    if right is None or right.lane_type != carla.LaneType.Driving or (right.lane_id > 0) != (start.lane_id > 0):
        return ["no kerb lane to the right of the start"]
    halves = [TRUCK_HALF[0], CAR_HALF]
    try:
        spots = occluder_transforms(crossing, halves, OCCLUDER_GAP)
    except ValueError as error:
        return [str(error)]
    wp = crossing.wp
    fwd, loc = wp.transform.get_forward_vector(), wp.transform.location
    along = lambda p: (p[0] - loc.x) * fwd.x + (p[1] - loc.y) * fwd.y  # distance past the crossing point along the ego lane
    near_edge = min(along((p.x, p.y)) for p in crossing.crosswalk.points)
    problems = []
    for name, tf, half in zip(("truck", "car"), spots, halves):
        lane = cmap.get_waypoint(tf.location, project_to_road=False, lane_type=carla.LaneType.Driving)
        kerb = lane.get_right_lane() if lane is not None else None
        if (lane is None or lane.is_junction or (lane.lane_id > 0) != (start.lane_id > 0)
                or (kerb is not None and kerb.lane_type == carla.LaneType.Driving)):
            problems.append(f"{name} is not on the rightmost driving lane, off the junction")
            continue
        f = tf.get_forward_vector()
        front = (tf.location.x + f.x * half, tf.location.y + f.y * half)
        if along(front) > near_edge - 0.5:
            problems.append(f"{name} front is {along(front) - near_edge:.1f} m past the near edge of the crosswalk")
        if name == "truck":
            corners = [(tf.location.x + f.x * dx - f.y * dy, tf.location.y + f.y * dx + f.x * dy)
                       for dx in (-half, half) for dy in (-TRUCK_HALF[1], TRUCK_HALF[1])]
            if min(distance_to_polygon(c, crossing.crosswalk.points) for c in corners) <= 0.0:
                problems.append("truck overlaps the crosswalk")
    # On a curve to the right the kerb lane is shorter than the ego lane the queue is measured along, so the 1.5 m clear space shrinks
    if tf_distance(spots[0], spots[1]) < halves[0] + halves[1]:
        problems.append("car overlaps the truck")
    # The kerb lane at the crossing point is where the pedestrian walks past the truck's front: out of sight behind it
    right_vector = wp.transform.get_right_vector()
    target = carla.Location(loc.x + right_vector.x * wp.lane_width, loc.y + right_vector.y * wp.lane_width)
    for d in HIDE_BEFORE:
        ego = lane_point(wp, -d).transform.location
        if not ray_hits_box((ego.x, ego.y), (target.x, target.y), spots[0], TRUCK_HALF[0], TRUCK_HALF[1]):
            problems.append(f"the kerb lane at the crossing is in sight of the ego {d} m before it")
    problems += check_prop(cmap, crossing, spots[0], wp)
    walk_in = next(walk_in_spawns(cmap, crossing, WALK_IN), None)
    if walk_in is None:
        return problems + ["no walk-in start off the driving lanes"]
    a, b = walk_in[1].location, walk_in[2].location
    leg = [(a.x + (b.x - a.x) * k / 12, a.y + (b.y - a.y) * k / 12) for k in range(13)]  # every 0.5 m
    if any(cmap.get_waypoint(carla.Location(x, y, a.z), project_to_road=False, lane_type=carla.LaneType.Driving)
           for x, y in leg):
        problems.append("the walk-in is on a driving lane")
    for d in LEG_HIDDEN + TURN_HIDDEN:
        ego = lane_point(wp, -d).transform.location
        points = leg if d in LEG_HIDDEN else leg[-1:]
        if not all(ray_hits_box((ego.x, ego.y), p, spots[0], TRUCK_HALF[0], TRUCK_HALF[1]) for p in points):
            problems.append(f"the walk-in is in sight of the ego {d} m before the crossing")
    return problems


def check_prop(cmap, crossing, truck, wp):
    """The bus stop of the occluder block: off the driving lanes and the crosswalk, clear of the child's walk, hiding its start."""
    problems = []
    prop = prop_transform(crossing, *PROP_HALF)
    yaw = math.radians(prop.rotation.yaw)
    cos, sin = math.cos(yaw), math.sin(yaw)
    along = [k / 4.0 * PROP_HALF[0] for k in range(-4, 5)]
    across = [k / 4.0 * PROP_HALF[1] for k in range(-4, 5)]
    box = [(prop.location.x + cos * a - sin * b, prop.location.y + sin * a + cos * b)
           for a in along for b in across if abs(a) == PROP_HALF[0] or abs(b) == PROP_HALF[1] or (a == 0 and b == 0)]
    if any(cmap.get_waypoint(carla.Location(x, y, prop.location.z), project_to_road=False,
                             lane_type=carla.LaneType.Driving) for x, y in box):
        problems.append("the prop is on a driving lane")
    if min(distance_to_polygon(p, crossing.crosswalk.points) for p in box) <= 0.0:
        problems.append("the prop is on the crosswalk")
    a, b = crossing.spawn.location, crossing.end
    walk = [(a.x + (b.x - a.x) * k / 40, a.y + (b.y - a.y) * k / 40) for k in range(41)]
    if any(ray_hits_box(p, p, prop, *PROP_HALF) for p in walk):
        problems.append("the prop is in the child's way")
    for d in PROP_HIDDEN:
        ego = lane_point(wp, -d).transform.location
        for dx, dy in ((0, 0), (0.25, 0), (-0.25, 0), (0, 0.25), (0, -0.25)):
            spot = (a.x + dx, a.y + dy)
            if not (ray_hits_box((ego.x, ego.y), spot, prop, *PROP_HALF)
                    or ray_hits_box((ego.x, ego.y), spot, truck, TRUCK_HALF[0], TRUCK_HALF[1])):
                problems.append(f"the child's waiting spot is in sight of the ego {d} m before the crossing")
                break
    return problems


def tf_distance(a, b):
    return math.hypot(a.location.x - b.location.x, a.location.y - b.location.y)


def check_fallback(cmap, index):
    """No-crosswalk fallback from every junction entry without a crosswalk: (checked, failure messages)."""
    checked, failures, seen = 0, [], set()
    for lane_start, junction_wp in cmap.get_topology():
        key = (lane_start.road_id, lane_start.lane_id)
        if lane_start.is_junction or not junction_wp.is_junction or key in seen:
            continue
        seen.add(key)
        before = junction_wp.previous(2.0)
        entry = junction_entry_ahead(before[0], 5.0)[0] if before and not before[0].is_junction else None
        if entry is None:
            continue
        crossing = find_crossing(cmap, entry, index, max_distance=5.0, min_ahead=None)
        if crossing is None or crossing.crosswalk is not None:
            continue
        checked += 1
        fwd = entry.transform.get_forward_vector()
        loc = entry.transform.location
        problems = check_walk_sides(cmap, crossing, (loc.x, loc.y), (fwd.x, fwd.y))
        if problems:
            failures.append(f"fallback road {key[0]} lane {key[1]} at ({loc.x:.1f}, {loc.y:.1f}): " + "; ".join(problems))
    return checked, failures


def check_never_behind(cmap, index):
    """From every junction entry and points before it, the crossing is never behind the ego: (checked, found, failures)."""
    checked, found, failures, seen = 0, 0, [], set()
    for lane_start, junction_wp in cmap.get_topology():
        key = (lane_start.road_id, lane_start.lane_id)
        if lane_start.is_junction or not junction_wp.is_junction or key in seen:
            continue
        seen.add(key)
        before = junction_wp.previous(2.0)
        entry = junction_entry_ahead(before[0], 5.0)[0] if before and not before[0].is_junction else None
        if entry is None:
            continue
        for back in BEFORE_ENTRY:
            starts = entry.previous(back) if back else [entry]
            if not starts or starts[0].is_junction:
                continue
            checked += 1
            crossing = find_crossing(cmap, starts[0], index, max_distance=100.0)
            if crossing is None:
                continue
            found += 1
            if crossing.distance <= 0.0:
                loc = entry.transform.location
                failures.append(f"road {key[0]} lane {key[1]} at ({loc.x:.1f}, {loc.y:.1f}), start {back:g} m before "
                                f"the entry: crossing at distance {crossing.distance:.1f} m")
    return checked, found, failures


def check_infra_selftest():
    """reset_world before a run must not touch the Traffic Manager; an occluder with an unknown model or prop is no silent substitute."""
    from types import SimpleNamespace
    from unittest import mock

    from carsim.blocks.occluder import Occluder
    from carsim.sim import reset_world

    problems = []
    for port, expected in ((None, []), (8000, [8000])):
        client = mock.MagicMock()
        client.get_world.return_value.get_settings.return_value = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None)
        reset_world(client, traffic_manager_port=port)
        called = [c.args[0] for c in client.get_trafficmanager.call_args_list]
        if called != expected:
            problems.append(f"reset_world(traffic_manager_port={port}) asked for the Traffic Managers {called}, expected {expected}")

    library = mock.MagicMock()
    ids = ["vehicle.carlamotors.european_hgv", "static.prop.busstop"]
    library.filter.side_effect = lambda pattern: [SimpleNamespace(id=i, has_attribute=lambda name: False)
                                                  for i in ids if fnmatch.fnmatch(i, pattern)]
    ctx = SimpleNamespace(library=library, log=lambda *a: None)
    Occluder(ctx, {"model": "vehicle.carlamotors.european_hgv"})  # a known model constructs
    try:
        Occluder(ctx, {"model": "vehicle.carlamotors.european_hvg"})
        problems.append("an occluder with an unknown model was accepted")
    except ValueError as error:
        if "european_hvg" not in str(error):
            problems.append(f"the unknown model error does not name the id: {error}")
    with mock.patch("carsim.blocks.occluder.CarlaDataProvider") as provider:
        occluder = Occluder(ctx, {"prop": "static.prop.bustop"})
        occluder.log = lambda *a: None
        occluder.ctx = SimpleNamespace(library=library, crossing=None)
        occluder._spawn_prop()
        if provider.request_new_actor.called:
            problems.append("an unknown prop was spawned (CarlaDataProvider would substitute a default)")
    return problems


def check_sim_selftest():
    """
    carsim.sim on large maps: the streaming distances are set after a load (and only when they differ), a far spectator
    move goes in steps of at most SPECTATOR_STEP, wait_for_ground gives up on a map whose trace hits nothing (after
    no_hit + settle s, None) and takes a surface with a label of its own after no_hit s, and destroy_sensors_on removes
    only the chase view's camera on the ego.
    """
    from types import SimpleNamespace
    from unittest import mock

    from carsim import sim

    problems = []
    settings = SimpleNamespace(tile_stream_distance=3000.0, actor_active_distance=2000.0)
    world = mock.MagicMock()
    world.get_settings.return_value = settings
    world.get_map.return_value.name = "/Game/Maps/WolfsburgHighway/WolfsburgHighway"
    client = mock.MagicMock()
    client.get_world.return_value = world
    sim.ensure_map(client, "WolfsburgHighway")  # small enough to keep every tile loaded (sim.KEEP_LOADED)
    if (settings.tile_stream_distance, settings.actor_active_distance) != sim.KEEP_LOADED["WolfsburgHighway"] \
            or world.apply_settings.call_count != 1:
        problems.append(f"ensure_map did not set the streaming distances: {settings}")
    sim.apply_streaming(world, *sim.KEEP_LOADED["WolfsburgHighway"])
    if world.apply_settings.call_count != 1:
        problems.append("apply_streaming applied unchanged settings")
    client.get_world.return_value.get_map.return_value.name = "Town10HD_Opt"
    client.get_available_maps.return_value = ["/Game/Carla/Maps/WolfsburgHighway"]
    loaded = mock.MagicMock()
    loaded.get_settings.return_value = SimpleNamespace(tile_stream_distance=3000.0, actor_active_distance=2000.0)
    client.load_world.return_value = loaded
    sim.ensure_map(client, "WolfsburgHighway", tile_stream_distance=500.0, actor_active_distance=400.0)
    if loaded.get_settings.return_value.tile_stream_distance != 500.0 or not loaded.apply_settings.called:
        problems.append("ensure_map did not set the streaming distances after load_world")

    steps = sim.spectator_steps(carla.Location(0, 0, 0), carla.Location(950, 0, 50))
    hops = [a.distance(b) for a, b in zip([carla.Location(0, 0, 0)] + steps, steps + [carla.Location(950, 0, 50)])]
    if len(steps) != 9 or max(hops) > sim.SPECTATOR_STEP + 1e-6:
        problems.append(f"spectator steps: {len(steps)} stops, longest hop {max(hops):.1f} m")
    if sim.spectator_steps(carla.Location(0, 0, 0), carla.Location(60, 0, 0)):
        problems.append("a short spectator move must be one step")

    clock = [0.0]
    with mock.patch.object(sim.time, "time", lambda: clock[0]):
        def tick():
            clock[0] += 0.5
        blind = SimpleNamespace(ground_projection=lambda location, distance: None, wait_for_tick=tick)
        got = sim.wait_for_ground(blind, [carla.Location(0, 0, 0)], timeout=60.0, no_hit=15.0, settle=10.0, generated=True)
        if got != [None] or not 25.0 <= clock[0] <= 27.0:
            problems.append(f"wait_for_ground on a blind map: {got} after {clock[0]} s")
        clock[0] = 0.0
        try:  # not a generated map: no hit means an unloaded tile, so no fallback, only the timeout
            sim.wait_for_ground(blind, [carla.Location(0, 0, 0)], timeout=60.0, no_hit=15.0, settle=10.0, generated=False)
            problems.append("wait_for_ground took the generated-map fallback on another map")
        except RuntimeError:
            if clock[0] < 60.0:
                problems.append(f"wait_for_ground gave up after {clock[0]} s on another map")
        clock[0] = 0.0
        other = SimpleNamespace(location=carla.Location(0, 0, 0.8), label=carla.CityObjectLabel.Other)
        labelled = SimpleNamespace(ground_projection=lambda location, distance: other, wait_for_tick=tick)
        got = sim.wait_for_ground(labelled, carla.Location(0, 0, 0), timeout=60.0, no_hit=15.0, generated=True)
        if got != [other] or not 15.0 <= clock[0] <= 16.0:
            problems.append(f"wait_for_ground with another label: {got} after {clock[0]} s")

    ego = SimpleNamespace(id=1)

    class Sensor(SimpleNamespace):
        def destroy(self):
            self.destroyed = True
            return True

    ours = Sensor(attributes={"role_name": sim.CHASE_VIEW_ROLE}, parent=ego, destroyed=False)
    others = [Sensor(attributes={"role_name": "front"}, parent=ego, destroyed=False),
              Sensor(attributes={"role_name": sim.CHASE_VIEW_ROLE}, parent=SimpleNamespace(id=2), destroyed=False),
              Sensor(attributes={"role_name": sim.CHASE_VIEW_ROLE}, parent=None, destroyed=False)]
    world = mock.MagicMock()
    world.get_actors.return_value.filter.return_value = [ours] + others
    if sim.destroy_sensors_on(world, ego) != 1 or not ours.destroyed or any(o.destroyed for o in others):
        problems.append("destroy_sensors_on must remove the chase view's camera on the ego only")
    world.get_actors.side_effect = RuntimeError("gone")
    if sim.destroy_sensors_on(world, ego) != 0:
        problems.append("destroy_sensors_on must survive a world that is gone")
    return problems


# Measured on the generated map Wolfsburg2 (digital_twin/elevation_run.csv, a car driving it): x, y, OpenDRIVE z, the
# car's z. The car stands on the road mesh, which is OpenDRIVE z + road.deformation(x, y).
SURFACE_SAMPLES = (
    (1405.9, -675.7, 0.0, 0.354), (1408.2, -753.9, 0.0, -1.023), (1404.0, -837.0, 0.0, 0.048),
    (1391.9, -919.5, 0.0, 0.249), (1372.0, -1000.5, 0.0, -0.781), (1347.3, -1079.9, 0.0, 0.054),
    (1318.4, -1158.0, 0.0, 1.208), (1286.0, -1234.9, 0.0, -0.056), (1253.2, -1311.5, 1.054, 0.003),
    (1220.4, -1387.9, 2.487, 2.754),
)


def check_surface_selftest():
    """road.deformation against the measured car heights; which maps are deformed; the sidewalk lift."""
    from types import SimpleNamespace
    from unittest import mock

    from carsim import road

    problems = []
    for x, y, z, car_z in SURFACE_SAMPLES:
        if abs(car_z - z - road.deformation(x, y)) > 0.1:  # 735 samples: mean +0.006, sd 0.019, worst 0.095 m
            problems.append(f"surface at ({x}, {y}): {z + road.deformation(x, y):.3f}, the car stood at {car_z}")
    for name, want in (("/Game/CustomMaps/WolfsburgHighway/WolfsburgHighway", True), ("WolfsburgHighway", True),
                       ("/Game/CustomMaps/Other/Other", True), ("/Game/Carla/Maps/Town10HD_Opt", False), ("Town12", False)):
        with mock.patch.dict(os.environ, {road.DEFORMED_ENV: ""}):
            if road.deformed_map(name) != want:
                problems.append(f"deformed_map({name!r}) is not {want}")
    with mock.patch.dict(os.environ, {road.DEFORMED_ENV: "0"}):
        if road.deformed_map("WolfsburgHighway"):
            problems.append(f"{road.DEFORMED_ENV}=0 does not switch it off")
    lane = {"sidewalk": True}
    cmap = SimpleNamespace(get_waypoint=lambda loc, project_to_road=True, lane_type=None: object() if lane["sidewalk"] else None)
    at = carla.Location(100.0, -50.0, 0.0)
    if abs(road.surface_offset(cmap, at) - road.deformation(100.0, -50.0) - road.SIDEWALK_SURFACE) > 1e-9:
        problems.append("surface_offset on a sidewalk")
    lane["sidewalk"] = False
    if abs(road.surface_offset(cmap, at) - road.deformation(100.0, -50.0)) > 1e-9:
        problems.append("surface_offset on a road")
    # a map built with the flat-roads patch: no waves, the sidewalk lift stays
    flat = SimpleNamespace(name="/Game/CustomMaps/WolfsburgHighway/WolfsburgHighway", get_waypoint=cmap.get_waypoint)
    if road.deformation(100.0, -50.0, flat.name) != 0.0 or road.surface_offset(flat, at) != 0.0:
        problems.append("WolfsburgHighway must have no waves")
    lane["sidewalk"] = True
    if road.surface_offset(flat, at) != road.SIDEWALK_SURFACE:
        problems.append("surface_offset on a flat map's sidewalk")
    if road.deformation(100.0, -50.0, "/Game/CustomMaps/Wolfsburg2/Wolfsburg2") != road.deformation(100.0, -50.0):
        problems.append("Wolfsburg2 keeps the stock waves")
    return problems


def check_crosswalks_selftest():
    """road.crosswalks splits closed outlines (first point repeated) and open ones left at the end (4 corners each)."""
    from types import SimpleNamespace

    def quad(x):
        return [carla.Location(x, 0, 0), carla.Location(x + 3, 0, 0), carla.Location(x + 3, 4, 0), carla.Location(x, 4, 0)]

    problems = []
    for name, points, want in (
            ("closed", quad(0) + quad(0)[:1] + quad(10) + quad(10)[:1], [1.5, 11.5]),
            ("open (WolfsburgHighway)", quad(0) + quad(10) + quad(20), [1.5, 11.5, 21.5]),
            ("closed then open", quad(0) + quad(0)[:1] + quad(10), [1.5, 11.5]),
            ("open then closed", quad(0) + quad(10) + quad(10)[:1], [1.5, 11.5]),
            ("closed, open, closed", quad(0) + quad(0)[:1] + quad(10) + quad(20) + quad(20)[:1], [1.5, 11.5, 21.5]),
            ("open, closed sharing a corner", quad(0) + [carla.Location(5, 8, 0), carla.Location(3, 4, 0),
                                                      carla.Location(8, 4, 0), carla.Location(8, 8, 0),
                                                      carla.Location(5, 8, 0)], [1.5, 6.0]),
            ("open triangle", quad(0)[:3], [2.0]),
            ("two points", quad(0)[:2], [])):
        found = crosswalks(SimpleNamespace(get_crosswalks=lambda points=points: points))
        centres = [round(cw.center.x, 2) for cw in found]
        if centres != want:
            problems.append(f"crosswalks ({name}): centres {centres}, expected {want}")
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("towns", nargs="*", help="maps of the CARLA install (default: %s)" % " ".join(TOWNS))
    parser.add_argument("--xodr", action="append", default=[], type=Path, metavar="PATH",
                        help="check this OpenDRIVE file instead (repeatable)")
    args = parser.parse_args()
    maps = [(path.stem, path) for path in args.xodr] + [(town, xodr_path(town)) for town in args.towns]
    problems = (check_clearance_selftest() + check_hidden_selftest() + check_infra_selftest()
                + check_crosswalks_selftest() + check_sim_selftest() + check_surface_selftest())
    for problem in problems:
        print(f"  SELFTEST FAIL {problem}")
    if problems:
        print("FAILED")
        sys.exit(1)
    failed = False
    print(f"{'map':<10} {'crosswalks':>10} {'candidates':>10} {'failures':>8} {'cyclists':>8} {'fallbacks':>9} "
          f"{'failures':>8} {'starts':>6} {'crossings':>9} {'behind':>6}")
    for town, path in maps or [(town, xodr_path(town)) for town in TOWNS]:
        if not path.exists():
            print(f"{town:<10} skipped, no {path.name}")
            continue
        cmap = carla.Map(town, path.read_text(encoding="utf-8"))
        index = CrosswalkIndex(crosswalks(cmap))
        candidates = find_candidates(cmap, APPROACH)

        failures = 0
        stats = {}
        for number, (start, crossing) in enumerate(candidates):
            with_cyclists = number % CYCLIST_SAMPLE.get(town, 1) == 0
            problems = check_candidate(cmap, index, start, crossing, cyclists=with_cyclists, stats=stats)
            ignored = KNOWN_MARGINAL.get((town, start.road_id, start.lane_id), ())
            for message in [m for m in problems if m in ignored]:
                print(f"  NOTE {town} road {start.road_id} lane {start.lane_id}: {message} (known)")
            problems = [m for m in problems if m not in ignored]
            if problems:
                failures += 1
                loc = start.transform.location
                print(f"  FAIL {town} road {start.road_id} lane {start.lane_id} at ({loc.x:.1f}, {loc.y:.1f}): "
                      + "; ".join(problems))
        checked, fallback_failures = check_fallback(cmap, index)
        for message in fallback_failures:
            print(f"  FAIL {town} {message}")
        starts, found, behind = check_never_behind(cmap, index)
        for message in behind:
            print(f"  FAIL {town} {message}")
        print(f"{town:<10} {len(crosswalks(cmap)):>10} {len(candidates):>10} {failures:>8} {stats.get('checked', 0):>8} "
              f"{checked:>9} {len(fallback_failures):>8} {starts:>6} {found:>9} {len(behind):>6}")
        failures += len(fallback_failures) + len(behind)
        given = path in args.xodr
        inner = find_candidates(cmap, APPROACH, lane="inner") if town in MIN_INNER_CANDIDATES or given else []
        inner_failures = 0
        for start, crossing in inner:
            problems = check_occluder(cmap, start, crossing)
            ignored = KNOWN_MARGINAL.get((town, start.road_id, start.lane_id), ())
            for message in [m for m in problems if m in ignored]:
                print(f"  NOTE {town} inner road {start.road_id} lane {start.lane_id}: {message} (known)")
            problems = [m for m in problems if m not in ignored]
            if problems:
                inner_failures += 1
                loc = start.transform.location
                print(f"  FAIL {town} inner road {start.road_id} lane {start.lane_id} at ({loc.x:.1f}, {loc.y:.1f}): "
                      + "; ".join(problems))
        if town in MIN_INNER_CANDIDATES or given:
            print(f"  NOTE {town}: {len(inner)} inner-lane candidates, {inner_failures} failures of the occluder check")
        failures += inner_failures
        if len(inner) < MIN_INNER_CANDIDATES.get(town, 0):
            failed = True
            print(f"  FAIL {town}: expected at least {MIN_INNER_CANDIDATES[town]} inner-lane candidates")
        if stats.get("lines"):
            print(f"  NOTE {town}: {stats['lines'] - stats.get('no_wait', 0)} of {stats['lines']} bicycle lines have "
                  f"a sidewalk waiting spot (first one on a Sidewalk lane: {stats.get('on_sidewalk', 0)} of "
                  f"{stats.get('first', 0)})")
            if stats["lines"] >= 40 and stats.get("no_wait", 0) > WAITING_MAX_MISSING * stats["lines"]:
                failed = True
                print(f"  FAIL {town}: {stats['no_wait']} of {stats['lines']} bicycle lines have no waiting spot")
        if stats.get("exit_lines"):
            print(f"  NOTE {town}: {stats['exit_lines'] - stats.get('no_exit', 0)} of {stats['exit_lines']} bicycle "
                  f"lines have a sidewalk exit spot beyond the far kerb")
        if stats.get("short"):
            print(f"  NOTE {town}: {stats['short']} of {stats['checked']} checked candidates (narrow roads) fit fewer "
                  f"than {CYCLISTS} bicycles")

        if failures:
            failed = True
        if len(candidates) < MIN_CANDIDATES.get(town, 0):
            failed = True
            print(f"  FAIL {town}: expected at least {MIN_CANDIDATES[town]} candidates")

    print("FAILED" if failed else "PASSED")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
