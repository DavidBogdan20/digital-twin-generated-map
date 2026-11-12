#!/usr/bin/env python
"""
Find ego start positions for the pedestrian crossing scenario: lanes that run
into a junction whose crosswalk the ego lane crosses, with the ego in the
rightmost driving lane so the pedestrian steps off on its right (--lane inner: in the lane left of it, for the
occluded crossing, where a truck stands in the rightmost lane).

    python apps/find_crossing_spots.py                     # list 10 candidates on the variant's map
    python apps/find_crossing_spots.py --near 120 -340     # sort by distance to a point
    python apps/find_crossing_spots.py --lane inner        # starts in the lane left of the kerb lane
    python apps/find_crossing_spots.py --plot              # PNG map of the town with every candidate numbered
    python apps/find_crossing_spots.py --tour              # fly the camera through all listed candidates
    python apps/find_crossing_spots.py --show 3            # move the camera to candidate 3 and mark it
    python apps/find_crossing_spots.py --apply 3           # write candidate 3 into the XML

In the world, green is the ego start with an arrow along its drive direction
and red is the pedestrian's walk; each is labelled with its candidate number.
The camera looks steeply down with the ego driving towards the top of the screen.

Each candidate is the junction entry of a lane moved back `--approach` metres
and re-checked with the scenario's own crossing computation (road.find_crossing).
That only validates the geometry: it does not check for props or other
actors in the way, so spawning the pedestrian can still fail. N in --show and
--apply refers to the listing produced by the same options, so pass the same
--near/--seed/--count/--approach/--lane (each row prints a command that does).
"""

import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import carla  # noqa: E402

from carsim.blocks.spec import CROSSING_BLOCKS  # noqa: E402
from carsim.paths import OUTPUTS_DIR  # noqa: E402
from carsim.road import CrosswalkIndex, crosswalks, find_crossing, junction_entry_ahead, straight_route  # noqa: E402
from carsim.scenario_files import config_file, set_ego_pose, variants  # noqa: E402
from carsim.sim import connect, ensure_async, ensure_map, focus_spectator, wait_for_ground  # noqa: E402


INNER_STRAIGHT = 40.0  # an inner lane must run straight on this far from the junction entry [m]
INNER_MAX_TURN = 30.0  # ... within this heading change [deg]


def is_inner_lane(entry, kerb):
    """
    True when `kerb`, the lane right of `entry`, is the rightmost driving lane and goes the same way, and `entry`
    goes straight through the junction (the ego passes the truck of the occluder in the kerb lane).
    """
    if kerb is None or kerb.lane_type != carla.LaneType.Driving or (kerb.lane_id > 0) != (entry.lane_id > 0):
        return False
    outer = kerb.get_right_lane()
    if outer is not None and outer.lane_type == carla.LaneType.Driving:
        return False
    ahead = straight_route(entry, INNER_STRAIGHT)
    if not ahead:
        return False
    turn = abs((ahead[-1].transform.rotation.yaw - entry.transform.rotation.yaw + 180.0) % 360.0 - 180.0)
    return turn < INNER_MAX_TURN


def find_candidates(cmap, approach, tolerance=2.0, lane="kerb"):
    """
    Returns [(start_waypoint, Crossing)]: ego starts about `approach` metres
    before a crosswalk at a junction, one per incoming lane. `lane` "kerb": the
    rightmost driving lane; "inner": the same-direction driving lane left of it.
    """
    index = CrosswalkIndex(crosswalks(cmap))
    seen = set()
    found = []
    # One (lane start, first junction waypoint) pair per successor, so lanes repeat
    for lane_start, junction_wp in cmap.get_topology():
        if lane_start.is_junction or not junction_wp.is_junction:
            continue
        key = (lane_start.road_id, lane_start.lane_id)
        if key in seen:
            continue
        seen.add(key)

        before = junction_wp.previous(2.0)
        if not before or before[0].is_junction:
            continue
        entry, _ = junction_entry_ahead(before[0], max_distance=5.0)
        if entry is None:
            continue
        # kerb: only the rightmost driving lane, where the pedestrian steps off the kerb
        right = entry.get_right_lane()
        if lane == "inner":
            if not is_inner_lane(entry, right):
                continue
        elif right is not None and right.lane_type == carla.LaneType.Driving:
            continue

        crossing = find_crossing(cmap, entry, index, max_distance=5.0, min_ahead=None)
        if crossing is None or crossing.crosswalk is None:
            continue
        crosswalk = crossing.crosswalk

        # previous() lists every predecessor; take the one whose forward path leads back to this crossing,
        # and re-run the scenario's own lookup from there to be sure
        for start in entry.previous(approach - crossing.distance):
            if start.is_junction:
                continue
            again = find_crossing(cmap, start, index, max_distance=approach + 50.0)
            if (again is not None and again.crosswalk is crosswalk
                    and abs(again.distance - approach) <= tolerance):
                found.append((start, again))
                break
    return found


def listing_command(parser, args):
    """The command line that reproduces this listing, so a row's N means the same thing again."""
    cmd = ["python", "apps/find_crossing_spots.py", "--scenario", args.scenario]
    if args.variant:
        cmd += ["--variant", args.variant]
    for name in ("host", "port"):
        if getattr(args, name) != parser.get_default(name):
            cmd += [f"--{name}", str(getattr(args, name))]
    cmd += ["--approach", f"{args.approach:g}", "--count", str(args.count)]
    if args.lane != "kerb":
        cmd += ["--lane", args.lane]
    cmd += ["--near", f"{args.near[0]:g}", f"{args.near[1]:g}"] if args.near else ["--seed", str(args.seed)]
    return " ".join(cmd)


def mark_candidate(world, number, start, crossing, life_time):
    """Draw the ego start (green, with an arrow towards the crossing) and the pedestrian's walk (red)."""
    debug = world.debug
    green, red = carla.Color(0, 255, 0), carla.Color(255, 0, 0)
    ego, cross = start.transform.location, crossing.wp.transform.location
    debug.draw_arrow(ego + carla.Location(z=0.5), cross + carla.Location(z=0.5), thickness=0.15,
                     arrow_size=0.6, color=green, life_time=life_time)
    debug.draw_string(ego + carla.Location(z=3.0), f"{number}: ego start", color=green, life_time=life_time)
    debug.draw_line(crossing.spawn.location, crossing.end + carla.Location(z=1.0), thickness=0.2,
                    color=red, life_time=life_time)
    debug.draw_string(crossing.spawn.location + carla.Location(z=2.0), f"{number}: pedestrian",
                      color=red, life_time=life_time)


def view_candidate(world, number, start, crossing, life_time):
    """Mark a candidate and put the camera above the middle of the ego start and the crossing."""
    mark_candidate(world, number, start, crossing, life_time)
    ego, cross = start.transform.location, crossing.wp.transform.location
    middle = carla.Location((ego.x + cross.x) / 2, (ego.y + cross.y) / 2, (ego.z + cross.z) / 2)
    focus_spectator(world, middle, height=max(60.0, 1.2 * crossing.distance), yaw=start.transform.rotation.yaw)
    try:
        wait_for_ground(world, ego, timeout=60.0)
    except RuntimeError as error:
        print(f"  warning: {error}")


def plot_candidates(cmap, candidates, path):
    """Save a top-down PNG of the whole map with every candidate numbered (CARLA's y axis points down)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    roads = [w.transform.location for w in cmap.generate_waypoints(30.0)]
    fig, ax = plt.subplots(figsize=(11, 11), dpi=110)
    ax.scatter([r.x for r in roads], [r.y for r in roads], s=0.4, c="0.7", linewidths=0)
    for i, (start, _) in enumerate(candidates):
        loc = start.transform.location
        ax.scatter([loc.x], [loc.y], s=40, c="red", zorder=3)
        ax.annotate(str(i), (loc.x, loc.y), xytext=(5, 5), textcoords="offset points", color="red",
                    fontsize=11, fontweight="bold", zorder=4)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]  (CARLA's y axis points down)")
    ax.set_title(f"{cmap.name.split('/')[-1]}: ego start of each candidate")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--scenario", default="pedestrian_crossing", help="scenario folder under scenarios/")
    parser.add_argument("--variant", help="variant name in the XML (default: first)")
    parser.add_argument("--approach", type=float, default=60.0, help="ego start distance before the crossing [m]")
    parser.add_argument("--lane", choices=("kerb", "inner"), default="kerb",
                        help="ego lane: the rightmost driving lane, or the one left of it (occluded crossing)")
    parser.add_argument("--near", type=float, nargs=2, metavar=("X", "Y"), help="sort by distance to this point")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0, help="shuffle seed when --near is not given")
    parser.add_argument("--show", type=int, metavar="N", help="mark candidate N in the world and move the camera there")
    parser.add_argument("--tour", type=float, nargs="?", const=8.0, metavar="SECONDS",
                        help="visit every listed candidate in turn, SECONDS each (default 8)")
    parser.add_argument("--plot", action="store_true",
                        help="save outputs/crossing_candidates.png: a map with every listed candidate numbered")
    parser.add_argument("--apply", type=int, metavar="N", help="write candidate N as the ego start into the XML")
    args = parser.parse_args()

    all_variants = variants(args.scenario)
    variant = next((v for v in all_variants if v.name == args.variant), None) if args.variant else all_variants[0]
    if variant is None:
        sys.exit(f"Variant '{args.variant}' not found in scenario '{args.scenario}'")

    client = connect(args.host, args.port)
    world = ensure_map(client, variant.town)
    ensure_async(world)
    cmap = world.get_map()

    print(f"Searching {variant.town} for junction crossings (this can take a while on large maps)...")
    candidates = find_candidates(cmap, args.approach, lane=args.lane)
    if args.near:
        nx, ny = args.near
        candidates.sort(key=lambda c: math.hypot(c[0].transform.location.x - nx, c[0].transform.location.y - ny))
    else:
        random.Random(args.seed).shuffle(candidates)
    candidates = candidates[:args.count]
    if not candidates:
        sys.exit("No candidates found.")

    print(f"\n{'N':>3}  {'x':>9} {'y':>9} {'z':>7} {'yaw':>7}   road/lane   to crossing")
    for i, (start, crossing) in enumerate(candidates):
        tf = start.transform
        print(f"{i:>3}  {tf.location.x:9.2f} {tf.location.y:9.2f} {tf.location.z:7.2f} {tf.rotation.yaw:7.1f}"
              f"   {start.road_id:>5}/{start.lane_id:<4}  {crossing.distance:6.1f} m")
        print(f"       {listing_command(parser, args)} --apply {i}")

    if args.show is not None:
        if not 0 <= args.show < len(candidates):
            sys.exit(f"--show {args.show}: the listing has candidates 0..{len(candidates) - 1}")
        start, crossing = candidates[args.show]
        view_candidate(world, args.show, start, crossing, life_time=180.0)
        print(f"\nCamera moved to candidate {args.show} (green = ego start and drive direction, red = pedestrian walk). "
              "The markers disappear after 3 minutes.")

    if args.tour is not None:
        print(f"\nTour: {len(candidates)} candidates, {args.tour:g} s each. Note the number you like, then use --apply N.")
        for i, (start, crossing) in enumerate(candidates):
            view_candidate(world, i, start, crossing, life_time=args.tour + 30.0)
            print(f"  candidate {i}: ({start.transform.location.x:.0f}, {start.transform.location.y:.0f})")
            end = time.time() + args.tour
            while time.time() < end:
                world.wait_for_tick()

    if args.plot:
        path = OUTPUTS_DIR / "crossing_candidates.png"
        plot_candidates(cmap, candidates, path)
        print(f"\nMap with the numbered candidates: {path}")

    if args.apply is not None:
        # The spots are crossing starts: for a variant without a crossing (cut_in) or with an occluder in the other lane they would break the tuned start
        if not any(tag in variant.params for tag in CROSSING_BLOCKS):
            sys.exit(f"--apply: '{variant.name}' has no crossing block ({', '.join(CROSSING_BLOCKS)}), so a crossing start does not fit it; nothing written")
        if ("occluder" in variant.params) != (args.lane == "inner"):
            sys.exit(f"--apply: '{variant.name}' {'has' if 'occluder' in variant.params else 'has no'} occluder, which needs --lane "
                     f"{'inner' if 'occluder' in variant.params else 'kerb'} (got --lane {args.lane}); nothing written")
        if not 0 <= args.apply < len(candidates):
            sys.exit(f"--apply {args.apply}: the listing has candidates 0..{len(candidates) - 1}")
        tf = candidates[args.apply][0].transform
        path = config_file(args.scenario)
        # ScenarioRunner adds 0.2 m when spawning; a little extra avoids spawning into the road surface
        set_ego_pose(path, variant.name, tf.location.x, tf.location.y, tf.location.z + 0.3, tf.rotation.yaw)
        print(f"\nWrote candidate {args.apply} as the ego start of '{variant.name}' in {path}")


if __name__ == "__main__":
    main()
