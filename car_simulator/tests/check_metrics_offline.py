#!/usr/bin/env python
"""
Offline check of the metrics block (src/carsim/blocks/metrics.py) and its plumbing, no CARLA server needed.

  - the pure parts: the gap between two oriented boxes, the time to collision (ahead and in the ego's width, closing
    speed, a target that moves along), the Accumulator over a sampled series (test speed, brake onset, min distance,
    min TTC), build_metrics (speed reduction with and without an impact, the driver's events win over the samples,
    every field null when nothing was measured, JSON-serialisable)
  - the Sampler against fake actors: it returns RUNNING for ever (also when an actor read fails), spawns its collision
    sensor in initialise(), turns the first contact into the impact, and finish() destroys the sensor and writes the file
    named by CARSIM_METRICS_FILE once (safe to repeat; no file without the variable or without a tick)
  - the plumbing: scenario_runner_env puts the file path into the environment (and drops a stale one), read_metrics
    and metrics_summary of run_config, and write_run_config's copy of the XML (it runs on Python 3.8, without
    ElementTree.indent): the drive, the stop file and the raised ego z of the variant only

    python tests/check_metrics_offline.py

Exits with 1 if any check fails.
"""

import json
import math
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import carla  # noqa: E402
import py_trees  # noqa: E402

from carsim import runner  # noqa: E402
from carsim.blocks import metrics  # noqa: E402
from carsim.run_config import metrics_summary, read_metrics, write_run_config  # noqa: E402
from srunner.scenariomanager.timer import GameTime  # noqa: E402

FAILURES = []
KEYS = {"test_speed", "first_visible", "first_hazard", "brake_onset", "emergency", "emergency_count", "impact", "speed_reduction",
        "min_distance", "min_ttc", "duration"}
EGO = (0.0, 0.0, 0.0, 2.3, 1.0)  # centre x, y, yaw, half length, half width of an ego heading along +x


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL " + message)
    return condition


def near(value, expected, tolerance=1e-6):
    return value is not None and abs(value - expected) <= tolerance


def check_geometry():
    print("box distance and time to collision")
    child = (10.0, 0.0, 0.0, 0.3, 0.3)
    check(near(metrics.box_distance(EGO, child), 10.0 - 0.3 - 2.3), "gap of two aligned boxes")
    check(metrics.box_distance(EGO, (2.0, 0.5, 0.0, 0.3, 0.3)) == 0.0, "overlapping boxes must have gap 0")
    check(metrics.box_distance(EGO, (2.3 + 0.3, 0.0, 0.0, 0.3, 0.3)) == 0.0, "touching boxes must have gap 0")
    check(near(metrics.box_distance(EGO, (0.0, 3.0, 0.0, 0.3, 0.3)), 3.0 - 0.3 - 1.0), "gap to a box beside")
    corner = metrics.box_distance(EGO, (2.3 + 1.0 + 0.3, 1.0 + 1.0 + 0.3, 0.0, 0.3, 0.3))
    check(near(corner, math.hypot(1.0, 1.0)), f"corner to corner gap {corner}")
    rotated = metrics.box_distance(EGO, (10.0, 0.0, math.radians(45), 0.5, 0.5))
    check(near(rotated, 10.0 - 0.5 * math.sqrt(2) - 2.3), f"rotated box gap {rotated}")

    still, run = (0.0, 0.0), (10.0, 0.0)
    check(near(metrics.time_to_collision(EGO, child, run, still), (10.0 - 0.3 - 2.3) / 10.0), "ttc to a standing child")
    check(metrics.time_to_collision(EGO, (10.0, 3.0, 0.0, 0.3, 0.3), run, still) is None, "a target beside the path has no ttc")
    check(metrics.time_to_collision(EGO, (10.0, 1.2, 0.0, 0.3, 0.3), run, still) is not None, "a target half in the width has a ttc")
    check(metrics.time_to_collision(EGO, (-10.0, 0.0, 0.0, 0.3, 0.3), run, still) is None, "a target behind has no ttc")
    check(metrics.time_to_collision(EGO, child, still, still) is None, "a stopped ego has no ttc")
    check(metrics.time_to_collision(EGO, (2.6, 0.0, 0.0, 0.3, 0.3), run, still) == 0.0, "ttc 0 when the front touches")
    check(near(metrics.time_to_collision(EGO, (12.3, 0.0, 0.0, 2.0, 1.0), run, (6.0, 0.0)), (12.3 - 2.0 - 2.3) / 4.0),
          "ttc to a slower car ahead uses the closing speed")
    check(metrics.time_to_collision(EGO, (12.3, 0.0, 0.0, 2.0, 1.0), (5.0, 0.0), (6.0, 0.0)) is None,
          "no ttc when the car ahead is faster")
    # C7: a turned target beside the front bumper, boxes apart: the gap is to the part inside the width band
    ego = (0.0, 0.0, 0.0, 2.4, 1.0)
    car = (4.0, 2.2, math.radians(-20), 1.9, 0.9)  # cut-in car, nearest corner (1.91, 2.0) is beside the bumper
    bike = (3.0, 1.45, math.radians(-15), 0.8, 0.35)
    for name, box in (("car", car), ("bicycle", bike)):
        ttc = metrics.time_to_collision(ego, box, (8.33, 0.0), (4.0, -1.5))
        check(metrics.box_distance(ego, box) > 0.2 and ttc is not None and ttc > 0.05, f"C7 {name} apart but ttc {ttc}")
    ttc = metrics.time_to_collision(ego, car, (10.0, 0.0), (0.0, 0.0))
    edge = metrics._clip_band(metrics.box_corners(car), 1.0)
    check(near(ttc, (min(p[0] for p in edge) - 2.4) / 10.0), f"C7 car gap from the clipped part: {ttc}")
    check(metrics.time_to_collision(ego, (6.0, 2.5, math.radians(-20), 1.0, 0.5), (10.0, 0.0), (0.0, 0.0)) is None,
          "C7 a turned target wholly outside the band has no ttc")
    check(metrics._clip_band([(1.0, 2.0), (3.0, 2.0), (3.0, 3.0)], 1.0) == [], "clip: nothing inside gives []")
    # an ego that is not at the origin and heads along +y
    ego_y = (5.0, 5.0, math.pi / 2, 2.3, 1.0)
    check(near(metrics.time_to_collision(ego_y, (5.0, 15.0, 0.0, 0.3, 0.3), (0.0, 10.0), still), (10.0 - 0.3 - 2.3) / 10.0),
          "ttc with a rotated ego")


def run_series(samples, events=None, impact=None, test_speed=None, impact_known=True):
    acc = metrics.Accumulator()
    for sample in samples:
        acc.add(*sample)
    return acc, metrics.build_metrics(acc, events, impact, test_speed, impact_known)


def check_accumulator():
    print("accumulator and build_metrics")
    v = 30 / 3.6
    # 1 s of cruising at 30 km/h, then braking from t=1.0: speed drops 8 m/s per s, the child is 20 m ahead at the start
    samples = [(0.05 * i, v, 0.0, 20.0 - v * 0.05 * i, (20.0 - v * 0.05 * i) / v) for i in range(20)]
    samples += [(1.0 + 0.05 * i, max(0.0, v - 8.0 * 0.05 * i), 1.0, 0.8, 0.5 + 0.01 * i) for i in range(10)]
    acc, result = run_series(samples)
    check(set(result) == KEYS, f"keys {set(result) ^ KEYS}")
    check(near(result["test_speed"], v, 1e-3), f"test speed is the speed before braking: {result['test_speed']}")
    onset = result["brake_onset"]
    check(onset is not None and near(onset["t"], 1.0) and near(onset["speed"], v, 1e-3), f"brake onset {onset}")
    check(near(onset["ttc"], 0.5) and near(onset["distance"], 0.8), f"brake onset ttc/distance {onset}")
    check(near(result["min_distance"], 0.8) and near(result["min_ttc"], 0.5), f"min distance/ttc {result}")
    check(near(result["duration"], 1.45), f"duration {result['duration']}")
    check(result["impact"]["happened"] is False and result["impact"]["speed"] is None, "no impact expected")
    check(near(result["speed_reduction"], v, 1e-3), "avoided: the speed reduction is the whole test speed")
    check(result["emergency_count"] == 0 and result["first_visible"] is None and result["emergency"] is None,
          "no driver events: null / 0")

    impact = {"t": 1.4, "speed": 4.0, "other_type": "walker.pedestrian.0012", "other_is_target": True}
    _, result = run_series(samples, impact=impact, test_speed=v)
    check(near(result["speed_reduction"], v - 4.0, 1e-3) and result["impact"]["happened"], "speed reduction at an impact")
    check(result["impact"]["other_type"] == "walker.pedestrian.0012" and result["impact"]["other_is_target"] is True,
          "impact fields")
    _, result = run_series(samples, impact={"t": 1.4, "speed": None, "other_type": None, "other_is_target": None})
    check(result["speed_reduction"] is None and result["impact"]["happened"], "impact with an unknown speed")

    # brake_onset fallback: a start-up cruise brake without a hazard (ttc null or >= HAZARD_TTC) is not an onset
    startup = [(0.0, 5.0, 0.0, 40.0, None), (0.25, 5.0, 0.6, 16.0, None), (0.5, 5.0, 0.6, 15.0, 9.0),
               (1.0, 5.0, 0.0, 12.0, 3.0), (1.5, 5.0, 0.5, 8.0, 3.0)]
    _, result = run_series(startup)
    check(result["brake_onset"] is not None and result["brake_onset"]["t"] == 1.5, f"fallback onset needs a hazard: {result['brake_onset']}")
    _, result = run_series(startup[:3])
    check(result["brake_onset"] is None and near(result["test_speed"], 5.0), f"start-up brake is no onset: {result['brake_onset']}")
    hazard = {"first_visible": {"t": 0.4, "ttc": None}}
    _, result = run_series(startup, hazard)
    check(result["brake_onset"] is None, "a driver that wrote events and no onset: the sampler fallback must not fill in")
    _, result = run_series(startup, {"first_hazard": {"t": 1.0, "actor": 3, "kind": "walker", "distance": 12.0, "ttc": 3.0}})
    check(result["first_hazard"] == {"t": 1.0, "actor": 3, "kind": "walker", "distance": 12.0, "ttc": 3.0}
          and result["brake_onset"] is None, f"first_hazard passes through: {result['first_hazard']}")
    _, result = run_series(samples)
    check(result["first_hazard"] is None, "no first_hazard: null")
    # C8: no collision sensor: the impact is unknown, not "no impact" with full credit
    _, result = run_series(samples, test_speed=v, impact_known=False)
    check(result["impact"]["happened"] is None and result["speed_reduction"] is None, f"unknown impact: {result['impact']}")

    events = {"first_visible": {"t": 0.4, "actor": 7, "kind": "walker", "distance": 9.123456, "ttc": 1.5},
              "brake_onset": {"t": 0.9, "speed": 8.0, "ttc": 1.1, "distance": 7.0},
              "emergency": {"t": 1.0, "speed": 8.0, "distance": 6.0}, "emergency_count": 2}
    _, result = run_series(samples, events)
    check(result["brake_onset"]["t"] == 0.9, "the driver's brake onset must win over the samples")
    check(result["first_visible"]["distance"] == 9.123 and result["emergency_count"] == 2, f"events {result}")

    _, result = run_series([(0.0, None, None)])
    check(result["test_speed"] is None and result["speed_reduction"] is None and result["min_distance"] is None
          and result["brake_onset"] is None, f"unreadable samples give null: {result}")
    _, empty = run_series([])
    check(set(empty) == KEYS and empty["duration"] is None and empty["test_speed"] is None, f"no samples: {empty}")
    check(json.loads(json.dumps(empty)) == empty, "metrics must survive JSON")
    check(near(run_series([(0.0, 5.0, 0.0)])[1]["speed_reduction"], 5.0), "no target: speed reduction from the samples")
    # the test speed stops growing once braking began
    _, late = run_series([(0.0, 5.0, 0.0), (0.1, 4.0, 0.5), (0.2, 9.0, 0.0)])
    check(near(late["test_speed"], 5.0), f"test speed after the brake onset: {late['test_speed']}")


# --- fakes for the sampler ------------------------------------------------------------------------------------------

class FakeSensor:
    def __init__(self):
        self.is_alive = True
        self.callback = None
        self.destroyed = 0

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        pass

    def destroy(self):
        self.destroyed += 1
        self.is_alive = False


class FakeVehicle:
    def __init__(self, type_id, x, y, vx, brake=0.0, half=(2.3, 1.0), actor_id=1):
        self.type_id = type_id
        self.id = actor_id
        self.x, self.y, self.vx, self.brake = x, y, vx, brake
        self.bounding_box = carla.BoundingBox(carla.Location(), carla.Vector3D(half[0], half[1], 0.8))
        self.fail = False

    def _check(self):
        if self.fail:
            raise RuntimeError("actor gone")

    def get_velocity(self):
        self._check()
        return carla.Vector3D(self.vx, 0.0, 0.0)

    def get_control(self):
        self._check()
        return carla.VehicleControl(brake=self.brake)

    def get_transform(self):
        self._check()
        return carla.Transform(carla.Location(self.x, self.y, 0.0), carla.Rotation())


def fake_context(target=True):
    sensor = FakeSensor()
    ego = FakeVehicle("vehicle.lincoln.mkz_2017", 0.0, 0.0, 8.0)
    child = FakeVehicle("walker.pedestrian.0012", 12.0, 0.0, 0.0, half=(0.3, 0.3), actor_id=9) if target else None
    logs = []
    world = SimpleNamespace(spawn_actor=lambda blueprint, transform, attach_to=None: sensor)
    ctx = SimpleNamespace(ego=ego, target=child, metrics={}, world=world,
                          library=SimpleNamespace(find=lambda name: name),
                          log=lambda who, message: logs.append((who, message)))
    return ctx, sensor, ego, child, logs


def clock(sampler, ticks, step=0.05, start=0.0):
    for i in range(ticks):
        with mock.patch.object(GameTime, "get_time", staticmethod(lambda: start + i * step)):
            status = sampler.update()
        if status != py_trees.common.Status.RUNNING:
            return status
    return py_trees.common.Status.RUNNING


def check_sampler():
    print("sampler")
    ctx, sensor, ego, child, logs = fake_context()
    block = metrics.Metrics(ctx, {})
    check(block.tag == "metrics" and block.from_start() == [block.sampler], "from_start gives the sampler")
    sampler = block.sampler
    sampler.initialise()
    check(sensor.callback == sampler._on_collision, "the collision sensor must be listened to")
    check(clock(sampler, 40) == py_trees.common.Status.RUNNING, "the sampler must never end")
    ego.fail = True  # an actor that cannot be read must not end or crash it
    check(clock(sampler, 3, start=2.0) == py_trees.common.Status.RUNNING, "the sampler ended on a failed read")
    check(sum("sampling failed" in m for _, m in logs) == 1, "a failed read is reported once")
    ego.fail = False
    ego.brake = 1.0
    clock(sampler, 3, start=3.0)
    check(sampler.acc.onset is not None and near(sampler.acc.onset["speed"], 8.0), f"brake onset {sampler.acc.onset}")
    check(near(sampler.acc.min_distance, 12.0 - 0.3 - 2.3), f"min distance {sampler.acc.min_distance}")
    check(near(sampler.acc.min_ttc, (12.0 - 0.3 - 2.3) / 8.0), f"min ttc {sampler.acc.min_ttc}")

    sampler._on_collision(SimpleNamespace(other_actor=child))
    sampler._on_collision(SimpleNamespace(other_actor=SimpleNamespace(type_id="vehicle.x", id=3)))
    check(sampler.impact["other_is_target"] is True and sampler.impact["other_type"] == child.type_id
          and near(sampler.impact["speed"], 8.0), f"first contact kept: {sampler.impact}")

    ctx.metrics.update({"first_visible": {"t": 0.5, "ttc": 1.2}, "emergency_count": 1})
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "sub" / "metrics.json"
        with mock.patch.dict(os.environ, {runner.METRICS_FILE_ENV: str(path)}):
            block.cleanup()
            check(sensor.destroyed == 1 and sampler.sensor is None, "the sensor must be destroyed by cleanup()")
            data = json.loads(path.read_text(encoding="utf-8"))
            check(KEYS <= set(data), f"file keys {KEYS - set(data)}")
            check(data["impact"]["happened"] and data["impact"]["other_is_target"], f"file impact {data['impact']}")
            check(data["first_visible"]["ttc"] == 1.2 and data["emergency_count"] == 1, "driver events in the file")
            check(data["target"] == child.type_id and data["test_speed"] == 8.0, f"file target / speed {data}")
            path.unlink()
            block.cleanup()  # again (ComposedScenario.__del__ does): no second write, no second destroy
            check(not path.exists() and sensor.destroyed == 1, "a repeated cleanup must do nothing")

    # no environment variable: no file, nothing crashes; no tick: nothing to write
    ctx, sensor, ego, child, logs = fake_context(target=False)
    block = metrics.Metrics(ctx, {})
    block.sampler.initialise()
    with tempfile.TemporaryDirectory() as tmp:
        with mock.patch.dict(os.environ, {runner.METRICS_FILE_ENV: str(Path(tmp) / "m.json")}):
            block.cleanup()
            check(not (Path(tmp) / "m.json").exists() and sensor.destroyed == 1, "no tick: no file, sensor destroyed")
        block2 = metrics.Metrics(fake_context(target=False)[0], {})
        clock(block2.sampler, 5)
        env = {k: v for k, v in os.environ.items() if k != runner.METRICS_FILE_ENV}
        with mock.patch.dict(os.environ, env, clear=True):
            block2.cleanup()
        check(block2.sampler.acc.min_distance is None, "no target: no distance")
    # no sensor: the sampler still samples
    ctx, sensor, ego, child, logs = fake_context()
    ctx.world = SimpleNamespace(spawn_actor=mock.Mock(side_effect=RuntimeError("no sensor")))
    sampler = metrics.Metrics(ctx, {}).sampler
    sampler.initialise()
    check(clock(sampler, 3) == py_trees.common.Status.RUNNING and sampler.acc.ticks == 3, "sampling without a sensor")
    check(sum("no collision sensor" in m for _, m in logs) == 1, "a failed spawn is logged once")
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "m.json"
        with mock.patch.dict(os.environ, {runner.METRICS_FILE_ENV: str(path)}):
            sampler.finish()
            data = json.loads(path.read_text(encoding="utf-8"))
        check(data["impact"]["happened"] is None and data["speed_reduction"] is None, f"C8 file: {data['impact']}")


def check_plumbing():
    print("environment and run.json helpers")
    path = Path(tempfile.gettempdir()) / "run" / "metrics.json"
    check(runner.scenario_runner_env(path)[runner.METRICS_FILE_ENV] == os.path.abspath(path), "metrics path in the env")
    with mock.patch.dict(os.environ, {runner.METRICS_FILE_ENV: "stale"}):
        check(runner.METRICS_FILE_ENV not in runner.scenario_runner_env(), "a stale path was passed on")
    check(metrics.METRICS_FILE_ENV == "CARSIM_METRICS_FILE", "the variable name changed")

    with tempfile.TemporaryDirectory() as tmp:
        file = Path(tmp) / "metrics.json"
        check(read_metrics(file) is None, "a missing file reads as None")
        file.write_text("{broken", encoding="utf-8")
        check(read_metrics(file) is None, "a broken file reads as None")
        file.write_text("[1]", encoding="utf-8")
        check(read_metrics(file) is None, "a non-object file reads as None")
        _, result = run_series([(0.0, 8.0, 0.0, 5.0, 0.6)], {"first_visible": {"ttc": 1.5}})
        file.write_text(json.dumps(result), encoding="utf-8")
        check(read_metrics(file) == result, "read_metrics returns the dict")
    line = metrics_summary({"impact": {"happened": True, "speed": 5.0}, "first_visible": {"ttc": 1.52},
                            "brake_onset": {"ttc": 0.94}, "min_distance": 0.0})
    check(line == "Metrics: impact YES at 18.0 km/h | first visible TTC 1.5 s | brake onset TTC 0.9 s | "
                  "min distance 0.0 m", line)
    line = metrics_summary({})
    check(line == "Metrics: impact unknown | first visible TTC n/a | brake onset TTC n/a | min distance n/a", line)
    line = metrics_summary({"impact": {"happened": False}})
    check(line.startswith("Metrics: impact no |"), line)

    import xml.etree.ElementTree as ET
    with tempfile.TemporaryDirectory() as tmp:
        source, copy = Path(tmp) / "s.xml", Path(tmp) / "copy.xml"
        source.write_text("""<scenarios>
            <scenario name="A_1" type="ComposedScenario" town="T"><ego_vehicle x="1" y="2" z="0.5" yaw="0"/>
                <weather cloudiness="20"/></scenario>
            <scenario name="B_1" type="ComposedScenario" town="T"><ego_vehicle x="1" y="2" z="0.5" yaw="0"/></scenario>
            </scenarios>""", encoding="utf-8")
        used = write_run_config(source, "A_1", "xml", copy, drive="autopilot", stop_file=Path(tmp) / "stop", ego_z=1.234)
        nodes = {n.get("name"): n for n in ET.parse(copy).getroot().iter("scenario")}
        a, b = nodes["A_1"], nodes["B_1"]
        check(used == {"cloudiness": 20.0}, f"weather values {used}")
        check(a.find("ego_vehicle").get("z") == "1.234" and b.find("ego_vehicle").get("z") == "0.5",
              "the raised ego z goes into the variant only")
        check(a.find("ego_control").get("mode") == "autopilot" and a.find("run_control") is not None,
              "drive and stop file in the copy")
        write_run_config(source, "A_1", "xml", copy)
        check(ET.parse(copy).getroot().find("scenario/ego_vehicle").get("z") == "0.5", "without ego_z the XML's z stays")


def main():
    check_geometry()
    check_accumulator()
    check_sampler()
    check_plumbing()
    print("FAILED" if FAILURES else "PASSED")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
