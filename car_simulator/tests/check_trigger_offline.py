#!/usr/bin/env python
"""
Offline check of the time-based start of the pedestrian (<pedestrian overlap="50"/>, carsim.road.impact_point), no CARLA
server needed. Checked:

  - impact_point on the 3 inner-lane starts of Town10HD (4 on CARLA 0.9.16): for 25, 50 and 75 % the impact lies on the walk line, that far
    (50 - overlap % of the ego width) right of the ego route, the 50 % one on the route's centre line, and the aim point
    is on the route, the ego's half length before the impact
  - a kinematic run, no braking: the ego at a constant 20, 30 and 40 km/h, the pedestrian starting when the condition
    of the block says so (the time to arrival at the aim point is less than the walk time, checked every 0.05 s tick,
    the way SR does): when the ego's front reaches the impact point the pedestrian is within 0.3 m of it
  - a mocked build of ComposedScenario (fakes of check_blocks_offline): with overlap the WaitForEgo node is an
    InTimeToArrivalToLocation and ctx.target the walker; without it (trigger_distance) the old
    InTriggerDistanceToLocation; the crowd variant (no overlap) keeps the distance trigger and the crowd starts under
    the same Triggered sequence
  - validation: overlap together with trigger_distance, and overlap above 100, are problems

    python tests/check_trigger_offline.py

Exits with 1 if any check fails.
"""

import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "apps"))

import carla  # noqa: E402

from check_blocks_offline import FAILURES, build, check, load_scenario_module, nodes, xodr_path  # noqa: E402
from carsim.blocks import spec  # noqa: E402
from carsim.blocks.pedestrian import EGO_HALF_EXTENT  # noqa: E402
from carsim.road import WALKER_ACCELERATION, impact_point  # noqa: E402
from carsim.scenario_files import COMPOSED_SCENARIO_FILE, config_file, variants  # noqa: E402
from find_crossing_spots import find_candidates  # noqa: E402
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTimeToArrivalToLocation,  # noqa: E402
                                                                               InTriggerDistanceToLocation)

HALF_WIDTH, HALF_LENGTH = EGO_HALF_EXTENT
WALK_SPEED = 1.39  # the NCAP child [m/s]
STEP = 0.05  # the simulation step [s]
EPS = 1e-3  # carla.Location holds floats [m]


def frame(crossing):
    tf = crossing.wp.transform
    return tf.location, tf.get_forward_vector(), tf.get_right_vector()


def check_geometry(candidates):
    print("impact_point")
    for start_wp, crossing in candidates:
        o, f, r = frame(crossing)
        a, end = crossing.spawn.location, crossing.end
        norm = math.hypot(end.x - a.x, end.y - a.y)
        lateral = {}
        for overlap in (25, 50, 75):
            impact, aim = impact_point(crossing, HALF_WIDTH, overlap, HALF_LENGTH)
            tag = f"({start_wp.transform.location.x:.0f}, {start_wp.transform.location.y:.0f}) {overlap} %"
            lateral[overlap] = (impact.x - o.x) * r.x + (impact.y - o.y) * r.y
            want = (50 - overlap) / 100 * 2 * HALF_WIDTH
            check(abs(lateral[overlap] - want) < EPS, f"{tag}: lateral {lateral[overlap]:.3f} m, want {want:.3f}")
            off_line = abs((impact.x - a.x) * (end.y - a.y) - (impact.y - a.y) * (end.x - a.x)) / norm
            ahead = (impact.x - a.x) * (end.x - a.x) + (impact.y - a.y) * (end.y - a.y)
            check(off_line < EPS and 0 < ahead < norm * norm, f"{tag}: the impact is not on the walk")
            aim_lateral = (aim.x - o.x) * r.x + (aim.y - o.y) * r.y
            gap = (impact.x - aim.x) * f.x + (impact.y - aim.y) * f.y
            check(abs(aim_lateral) < EPS and abs(gap - HALF_LENGTH) < EPS,
                  f"{tag}: aim {aim_lateral:.3f} m off the route, {gap:.3f} m before the impact")
        check(abs(lateral[50]) < EPS and lateral[25] > 0 > lateral[75], f"overlap order of the lateral offsets {lateral}")


def check_kinematics(candidates):
    print("kinematic run without braking")
    for start_wp, crossing in candidates:
        o, f, _ = frame(crossing)
        a = crossing.spawn.location
        ux, uy = crossing.end.x - a.x, crossing.end.y - a.y
        norm = math.hypot(ux, uy)
        for overlap in (25, 50, 75):
            impact, aim = impact_point(crossing, HALF_WIDTH, overlap, HALF_LENGTH)
            walk_time = math.hypot(impact.x - a.x, impact.y - a.y) / WALK_SPEED + WALK_SPEED / (2.0 * WALKER_ACCELERATION)
            aim_x = (aim.x - o.x) * f.x + (aim.y - o.y) * f.y  # along the route from crossing.wp
            for kmh in (20, 30, 40):
                v = kmh / 3.6
                x, t, start = -100.0, 0.0, None  # the ego centre along the route, 100 m before the crossing (the walk takes 6 s, 71 m at 40 km/h)
                while start is None and t < 60.0:
                    if (aim_x - x) / v < walk_time:  # InTimeToArrivalToLocation: distance / speed < time
                        start = t
                    else:
                        t += STEP
                        x += v * STEP
                tag = f"({start_wp.transform.location.x:.0f}, {start_wp.transform.location.y:.0f}) {overlap} % at {kmh} km/h"
                check(start is not None, f"{tag}: the walk never starts")
                if start is None:
                    continue
                tau = (aim_x - x) / v  # the walk time, from its start until the ego's front is at the impact
                ramp = WALK_SPEED / WALKER_ACCELERATION  # the walker starts from rest
                moved = WALKER_ACCELERATION * tau ** 2 / 2 if tau < ramp else WALK_SPEED * tau - WALK_SPEED * ramp / 2
                px, py = a.x + ux / norm * moved, a.y + uy / norm * moved
                miss = math.hypot(px - impact.x, py - impact.y)
                check(miss < 0.3, f"{tag}: the pedestrian is {miss:.2f} m from the impact point when the ego arrives")


def check_build(module, cmap, start, tmp):
    print("mocked build")
    pose = f'x="{start.location.x}" y="{start.location.y}" z="{start.location.z + 0.3}" yaw="{start.rotation.yaw}"'
    xml = Path(tmp) / "occluded.xml"
    xml.write_text(config_file("occluded_crossing").read_text(encoding="utf-8").replace('x="0" y="0" z="0" yaw="0"', pose),
                   encoding="utf-8")

    def triggers(scenario):
        return [n for n in nodes(scenario.scenario_tree) if n.name == "WaitForEgo"]

    child, crowd = [v.name for v in variants("occluded_crossing")][:2]
    scenario, world, spawns, config = build(module, child, xml, cmap)
    waits = triggers(scenario)
    check(len(waits) == 1 and isinstance(waits[0], InTimeToArrivalToLocation), f"child: trigger nodes {waits}")
    pedestrian = config.other_parameters["pedestrian"]
    check("overlap" in pedestrian and "trigger_distance" not in pedestrian, "the child needs overlap, no trigger_distance")
    walker = [b for b in scenario._blocks if b.tag == "pedestrian"][0].walker
    check(walker is not None and scenario._ctx.target is walker, "ctx.target is not the pedestrian's walker")
    scenario.other_actors.clear()

    def distance(config):
        del config.other_parameters["pedestrian"]["overlap"]
        config.other_parameters["pedestrian"]["trigger_distance"] = "47"

    scenario = build(module, child, xml, cmap, edit=distance)[0]
    waits = triggers(scenario)
    check(len(waits) == 1 and isinstance(waits[0], InTriggerDistanceToLocation), f"no overlap: trigger nodes {waits}")
    scenario.other_actors.clear()

    scenario = build(module, crowd, xml, cmap)[0]
    waits = triggers(scenario)
    check(len(waits) == 1 and isinstance(waits[0], InTriggerDistanceToLocation), f"crowd: trigger nodes {waits}")
    check(any(n.name == "TriggeredBehaviours" for n in nodes(scenario.scenario_tree)), "crowd: no triggered behaviours")
    scenario.other_actors.clear()


def check_validation():
    print("validation")
    ok = spec.check_params({"pedestrian": {"age": "child", "overlap": "50"}})
    check(not ok, f"overlap alone: {ok}")
    both = spec.check_params({"pedestrian": {"overlap": "50", "trigger_distance": "30"}})
    check(any("overlap" in p and "trigger_distance" in p for p in both), f"overlap and trigger_distance: {both}")
    big = spec.check_params({"pedestrian": {"overlap": "120"}})
    check(any("overlap" in p for p in big), f"overlap 120: {big}")


def main():
    cmap = carla.Map("Town10HD", xodr_path("Town10HD").read_text(encoding="utf-8"))
    candidates = find_candidates(cmap, 60.0, lane="inner")
    check(len(candidates) == 3, f"{len(candidates)} inner-lane starts on Town10HD, expected 3 (CARLA 0.9.15)")
    check_geometry(candidates)
    check_kinematics(candidates)
    module = load_scenario_module(COMPOSED_SCENARIO_FILE)
    with tempfile.TemporaryDirectory() as tmp:
        check_build(module, cmap, candidates[0][0].transform, tmp)
    check_validation()

    print("FAILED" if FAILURES else "PASSED")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
