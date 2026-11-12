#!/usr/bin/env python
"""
Offline check of the chase-view window's pure parts (apps/chase_view.py) and its launcher (carsim.runner), no CARLA
server and no display needed. Checked:

  - nearest_ahead: the nearest of several in the corridor wins, the distance is from the front bumper, an actor
    behind, beside the corridor or beyond 50 m is ignored, the ego itself and non-traffic actors are ignored, the
    corridor follows the ego's yaw, the kind is vehicle / walker / bicycle
  - TTC: gap over closing speed along the heading, None for a lead that is as fast or faster
  - hud_lines: speed, target ("-" when none), time, map, view and the key legend
  - parse_res, and chase_view_command_args (host and port only when not the defaults, --control with a folder,
    --spectator)
  - the flags run_scenario passes exist in ScenarioRunner v0.9.15's scenario_runner.py and manual_control.py parsers
    (read from their add_argument calls), and the driving window's set_autopilot is pinned to our Traffic Manager port
  - --spectator: follow_transform puts the spectator behind / above the hero in each view, turned with it, and
    SpectatorView moves the spectator and spawns no sensor; runner.start_window on Linux: its own session, the output
    in the log file, a warning without DISPLAY
  - the control panel: key and button hit-testing, the banner text and colour, the request / result files
    (write, read, clear, quit wins over restart), ScenarioRunner's result line, the watcher thread and the wait,
    and next_step (restart / quit / end / wait)

    python tests/check_chase_view_offline.py

Exits with 1 if any check fails.
"""

import argparse
import math
import os
import sys
import json
import tempfile
import threading
from collections import namedtuple
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # before pygame is imported by chase_view

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "apps"))

import chase_view  # noqa: E402
from carsim import runner  # noqa: E402
from carsim.runner import chase_view_command_args  # noqa: E402

FAILURES = []

V = namedtuple("V", "x y z")
Loc = namedtuple("Loc", "x y z")
Rot = namedtuple("Rot", "yaw")
Tf = namedtuple("Tf", "location rotation")
Box = namedtuple("Box", "extent")


class Fake:
    """Enough of a carla.Actor for nearest_ahead."""
    count = 0

    def __init__(self, type_id, x, y, yaw=0.0, vx=0.0, vy=0.0, half_length=2.4, wheels=None):
        Fake.count += 1
        self.id = Fake.count
        self.type_id = type_id
        self.attributes = {} if wheels is None else {"number_of_wheels": str(wheels)}
        self._tf = Tf(Loc(x, y, 0.0), Rot(yaw))
        self._v = V(vx, vy, 0.0)
        self.bounding_box = Box(V(half_length, 1.0, 0.8))

    def get_transform(self):
        return self._tf

    def get_velocity(self):
        return self._v


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL " + message)
    return condition


def car(x, y, **kw):
    return Fake("vehicle.tesla.model3", x, y, **kw)


def check_nearest_ahead():
    print("nearest_ahead")
    ego = car(0, 0, vx=10.0)  # 10 m/s along +x, front bumper at 2.4 m
    near, far = car(20, 0.5), car(40, 0)
    got = chase_view.nearest_ahead(ego, [far, near])
    check(got is not None and abs(got.distance - 17.6) < 1e-9, f"nearest not picked / distance from the bumper: {got}")
    check(got.kind == "vehicle", f"kind {got.kind}")
    check(chase_view.nearest_ahead(ego, [ego]) is None, "the ego is its own target")
    check(chase_view.nearest_ahead(ego, []) is None, "nothing around should give None")
    for name, other in (("behind", car(-10, 0)), ("beside the corridor", car(20, 1.5)),
                        ("beyond 50 m", car(55, 0)), ("not traffic", Fake("static.prop.box", 10, 0)),
                        ("inside the ego's length", car(1, 0))):
        check(chase_view.nearest_ahead(ego, [other]) is None, f"{name} was picked")
    check(chase_view.nearest_ahead(ego, [car(52.3, 0)]) is not None, "an actor 49.9 m from the bumper was dropped")
    walker = Fake("walker.pedestrian.0001", 15, -1.0)
    check(chase_view.nearest_ahead(ego, [walker]).kind == "walker", "walker kind")
    bike = Fake("vehicle.diamondback.century", 15, 0, wheels=2)
    check(chase_view.nearest_ahead(ego, [bike]).kind == "bicycle", "bicycle kind")
    # corridor follows the heading: ego facing +y (yaw 90), target straight ahead in y
    north = car(0, 0, yaw=90.0, vy=5.0)
    check(chase_view.nearest_ahead(north, [car(0, 20)]) is not None, "yaw 90: target ahead was missed")
    check(chase_view.nearest_ahead(north, [car(20, 0)]) is None, "yaw 90: target to the side was picked")


def check_ttc():
    print("TTC")
    ego = car(0, 0, vx=10.0)
    slow = chase_view.nearest_ahead(ego, [car(32.4, 0, vx=5.0)])  # gap 30 m, closing 5 m/s
    check(slow.ttc is not None and abs(slow.ttc - 6.0) < 1e-9, f"slow lead ttc {slow.ttc}")
    still = chase_view.nearest_ahead(ego, [Fake("walker.pedestrian.0001", 32.4, 0)])  # standing: closing 10 m/s
    check(abs(still.ttc - 3.0) < 1e-9, f"standing walker ttc {still.ttc}")
    check(chase_view.nearest_ahead(ego, [car(30, 0, vx=10.0)]).ttc is None, "same speed lead has a ttc")
    check(chase_view.nearest_ahead(ego, [car(30, 0, vx=15.0)]).ttc is None, "faster lead has a ttc")
    stopped_ego = car(0, 0)
    check(chase_view.nearest_ahead(stopped_ego, [car(30, 0)]).ttc is None, "both standing has a ttc")


def check_hud_lines():
    print("hud_lines")
    lines = chase_view.hud_lines(48.6, 12.34, "Town10HD_Opt", "chase", None)
    check(lines[0] == "49 km/h", f"speed line {lines[0]!r}")
    check(lines[1] == "Ahead: -   TTC -", f"no target {lines[1]!r}")
    check("12.3 s" in lines[2] and "Town10HD_Opt" in lines[2] and "chase" in lines[2], f"time/map/view {lines[2]!r}")
    check(lines[-1] == chase_view.LEGEND and "C" in lines[-1] and "H" in lines[-1] and "Esc" in lines[-1],
          f"legend {lines[-1]!r}")
    both = chase_view.hud_lines(0, 0, "m", "bonnet", chase_view.Ahead("walker", 17.64, 2.06))
    check(both[1] == "Ahead: walker 17.6 m   TTC 2.1 s", f"target line {both[1]!r}")
    none_ttc = chase_view.hud_lines(0, 0, "m", "bonnet", chase_view.Ahead("vehicle", 5.0, None))
    check(none_ttc[1] == "Ahead: vehicle 5.0 m   TTC -", f"target without ttc {none_ttc[1]!r}")


def check_args():
    print("parse_res / command args")
    check(chase_view.parse_res("1280x720") == (1280, 720), "parse_res 1280x720")
    check(chase_view.parse_res("800X600") == (800, 600), "parse_res is case-insensitive")
    for bad in ("1280", "axb", "0x720", "1x2x3"):
        try:
            chase_view.parse_res(bad)
            check(False, f"parse_res accepted {bad!r}")
        except argparse.ArgumentTypeError:
            pass
    check(chase_view_command_args() == [], "default args not empty")
    check(chase_view_command_args("10.0.0.5", 2100) == ["--host", "10.0.0.5", "--port", "2100"], "host/port args")
    check(chase_view_command_args(port=2100) == ["--port", "2100"], "port-only args")
    check(chase_view_command_args(control_dir="C:/tmp/c") == ["--control", "C:/tmp/c"], "control-only args")
    check(chase_view_command_args("10.0.0.5", 2100, "d") == ["--control", "d", "--host", "10.0.0.5", "--port", "2100"],
          "all args")
    check(chase_view_command_args(control_dir="d", spectator=True) == ["--control", "d", "--spectator"], "spectator arg")


def check_panel():
    print("buttons, keys, banner")
    import pygame
    check(chase_view.key_action(pygame.K_r) == "restart" and chase_view.key_action(pygame.K_ESCAPE) == "quit"
          and chase_view.key_action(pygame.K_c) is None, "key actions")
    for ended in (False, True):
        rects = chase_view.button_rects((1280, 720), ended)
        for name, (x, y, w, h) in rects.items():
            check(0 <= x and x + w <= 1280 and 0 <= y and y + h <= 720, f"{name} button is inside the window (ended={ended})")
            check(chase_view.hit_button((x, y), rects) == name, f"{name} hit at its corner (ended={ended})")
            check(chase_view.hit_button((x + w // 2, y + h // 2), rects) == name, f"{name} hit in the middle (ended={ended})")
            check(chase_view.hit_button((x + w, y + h // 2), rects) in (None, "quit"), f"{name}: the right edge is outside")
            check(chase_view.hit_button((x + w // 2, y + h), rects) is None, f"{name}: the bottom edge is outside")
        check(chase_view.hit_button((5, 360), rects) is None, f"a click elsewhere hit a button (ended={ended})")
        a, b = rects["restart"], rects["quit"]
        check(a[0] + a[2] <= b[0], f"the buttons overlap (ended={ended})")
    check(chase_view.button_rects((1280, 720), True) != chase_view.button_rects((1280, 720), False), "layout is the same")
    avoided = {"test_speed": 6.39, "first_visible": {"t": 5.0, "kind": "walker", "distance": 11.2, "ttc": 1.7},
               "brake_onset": {"t": 5.0, "speed": 6.4, "ttc": 1.7}, "emergency": None,
               "impact": {"happened": False}, "min_distance": 4.13, "speed_reduction": 6.39}
    head, detail, colour = chase_view.banner_lines({"result": "SUCCESS"}, metrics=avoided)
    check(head == "PASSED: no collision" and colour == chase_view.BANNER_COLORS["SUCCESS"], f"success banner {head!r}")
    check(detail == ["Test speed: 23 km/h",
                     "First saw the pedestrian 11.2 m ahead, 1.7 s before the car would have reached it",
                     "Started braking 0.0 s after seeing it, 1.7 s before reaching it",
                     "No emergency braking: a normal stop",
                     "Closest distance to the pedestrian: 4.1 m"], f"success banner lines {detail}")
    hit = {"test_speed": 8.33, "first_visible": {"t": 2.0, "kind": "bicycle", "distance": 6.0, "ttc": 0.7},
           "brake_onset": {"t": 2.3, "speed": 8.3, "ttc": 0.4}, "emergency": {"speed": 8.0},
           "impact": {"happened": True, "speed": 5.0, "other_is_target": True}, "speed_reduction": 3.33}
    head, detail, colour = chase_view.banner_lines({"result": "FAILURE"}, metrics=hit)
    check(head == "COLLISION: hit the cyclist at 18 km/h" and colour == chase_view.BANNER_COLORS["FAILURE"],
          f"collision banner {head!r}")
    check("Started braking 0.3 s after seeing it, 0.4 s before reaching it" in detail
          and "Emergency braking at 29 km/h" in detail and "Slowed down by 12 km/h before the impact" in detail,
          f"collision banner lines {detail}")
    head, detail, colour = chase_view.banner_lines({"result": "failure", "summary": None})
    check(head == "FAILED (failure)" and detail == [] and colour == chase_view.BANNER_COLORS["FAILURE"],
          f"failure banner without metrics {head!r} {detail}")
    for unknown in (None, {}, {"result": None, "summary": "Metrics: x"}):
        head, detail, colour = chase_view.banner_lines(unknown)
        check(head == "Run ended" and colour == chase_view.GREY, f"unknown result banner {head!r}")
    check(chase_view.banner_lines({"result": None, "summary": "Metrics: x"})[1] == ["Metrics: x"],
          "without a metrics file the console line is shown")
    with tempfile.TemporaryDirectory() as folder:  # read from the run folder named in result.json
        (Path(folder) / "metrics.json").write_text(json.dumps(avoided), encoding="utf-8")
        check(chase_view.banner_lines({"result": "SUCCESS", "run_dir": folder})[0] == "PASSED: no collision",
              "metrics.json in the run folder not read")
    check(chase_view.banner_lines({"result": "SUCCESS"}, "restart")[0] == "Restarting...", "restarting banner")
    pygame.init()  # the drawing runs on a plain surface (SDL dummy driver): no exception, something drawn
    surface, font = pygame.Surface((1280, 720)), pygame.font.Font(None, 22)
    surface.fill((200, 200, 200))
    chase_view.draw_banner(surface, pygame.font.Font(None, 44), font, (1280, 720), chase_view.banner_lines(None))
    chase_view.draw_buttons(surface, font, chase_view.button_rects((1280, 720), True), ("restart",))
    check(surface.get_at((640, 720 - chase_view.BUTTON_SIZE[1] - 80)) != (200, 200, 200, 255), "the banner drew nothing")


def check_control_files():
    print("request / result files")
    with tempfile.TemporaryDirectory() as folder:
        check(runner.pending_request(folder) is None, "a fresh folder has a request")
        check(runner.read_result(folder) is None, "a fresh folder has a result")
        runner.write_request(folder, runner.RESTART_REQUEST)
        check(runner.pending_request(folder) == "restart", "restart request not seen")
        check(runner.pending_request(folder) == "restart", "reading a request must not consume it")
        runner.write_request(folder, runner.QUIT_REQUEST)
        check(runner.pending_request(folder) == "quit", "quit must win over restart")
        runner.clear_request(folder, runner.QUIT_REQUEST)
        runner.clear_request(folder, runner.RESTART_REQUEST)
        runner.clear_request(folder, runner.RESTART_REQUEST)  # clearing nothing is fine
        check(runner.pending_request(folder) is None, "cleared request still pending")
        runner.write_result(folder, 2, "FAILURE", "Metrics: impact YES", "outputs/x")
        got = runner.read_result(folder)
        check(got == {"run": 2, "result": "FAILURE", "summary": "Metrics: impact YES", "run_dir": "outputs/x"}, f"result {got}")
        runner.write_result(folder, 3, None, None, "outputs/y")
        check(runner.read_result(folder)["result"] is None, "an unknown result must stay null")
        runner.clear_result(folder)
        check(runner.read_result(folder) is None, "cleared result still read")
        (runner.Path(folder) / runner.RESULT_FILE).write_text("{half", encoding="utf-8")
        check(runner.read_result(folder) is None, "a broken result file must read as None")
        (runner.Path(folder) / runner.RESULT_FILE).write_text("[1]", encoding="utf-8")
        check(runner.read_result(folder) is None, "a result that is not an object must read as None")

        # the watcher: calls back once when a request appears, nothing after done is set
        seen, done = [], threading.Event()
        thread = threading.Thread(target=runner.watch_requests, args=(folder, seen.append, done, 0.01))
        thread.start()
        runner.write_request(folder, runner.RESTART_REQUEST)
        thread.join(2)
        check(seen == ["restart"] and not thread.is_alive(), f"watcher saw {seen}")
        quiet, stop = [], threading.Event()
        runner.clear_request(folder, runner.RESTART_REQUEST)
        thread = threading.Thread(target=runner.watch_requests, args=(folder, quiet.append, stop, 0.01))
        thread.start()
        stop.set()
        thread.join(2)
        check(quiet == [] and not thread.is_alive(), "the watcher did not stop when the run was over")

        # the wait: a request, or the window gone (= quit)
        runner.write_request(folder, runner.RESTART_REQUEST)
        check(runner.wait_for_request(folder, lambda: True, 0.01) == "restart", "wait did not return the request")
        runner.clear_request(folder, runner.RESTART_REQUEST)
        check(runner.wait_for_request(folder, lambda: False, 0.01) == "quit", "a window that is gone must mean quit")
        alive = iter([True, True, False])
        check(runner.wait_for_request(folder, lambda: next(alive), 0.01) == "quit", "wait did not poll the window")


def check_run_loop():
    print("result line / next_step")
    table = " ======= Results of Scenario: CutIn_Parking_Town10HD_1 ---- SUCCESS ======="
    check(runner.parse_result_line(table + "\n") == "SUCCESS", "SUCCESS line")
    check(runner.parse_result_line(table.replace("SUCCESS", "FAILURE")) == "FAILURE", "FAILURE line")
    for other in ("", "ScenarioManager: Running scenario", "| GLOBAL RESULT | SUCCESS |", "Results of Scenario: x"):
        check(runner.parse_result_line(other) is None, f"{other!r} parsed as a result")
    step = runner.next_step
    check(step("restart", False, True) == "restart" and step("restart", False, False) == "restart", "restart request")
    check(step("quit", False, True) == "quit" and step("quit", False, False) == "quit", "quit request")
    check(step(None, False, True) == "wait", "window open, no request: wait")
    check(step(None, False, False) == "end", "no window: a single run")
    for request in (None, "restart", "quit"):
        check(step(request, True, True) == "end", f"Ctrl-C must end everything (request {request})")


def check_camera_never_orphaned():
    print("camera cleanup")

    class Sensor:
        def __init__(self):
            self.destroyed = False

        def listen(self, callback):
            pass

        def stop(self):
            pass

        def destroy(self):
            self.destroyed = True

    class Blueprint:
        def set_attribute(self, name, value):
            pass

    class World:
        def __init__(self):
            self.sensors = []

        def get_blueprint_library(self):
            return type("Library", (), {"find": lambda _, name: Blueprint()})()

        def spawn_actor(self, blueprint, transform, attach_to=None, attachment_type=None):
            self.sensors.append(Sensor())
            return self.sensors[-1]

    class Hero(Fake):
        alive = True

        def get_transform(self):
            if not self.alive:
                raise RuntimeError("trying to operate on a destroyed actor")
            return super().get_transform()

    world, hero = World(), Hero("vehicle.lincoln.mkz_2017", 0, 0)
    camera = chase_view.Camera(world, hero, (320, 240))
    camera.set_view(0)
    check(camera.sensor is world.sensors[0] and not world.sensors[0].destroyed, "a camera on a live hero stays")
    camera.set_view(1)
    check(world.sensors[0].destroyed and camera.sensor is world.sensors[1], "changing the view destroys the old camera")
    hero.alive = False  # the hero is destroyed while the camera is being (re)spawned
    try:
        camera.set_view(2)
        check(False, "set_view on a destroyed hero must raise RuntimeError")
    except RuntimeError:
        pass
    check(all(sensor.destroyed for sensor in world.sensors) and camera.sensor is None,
          "no camera is left when the hero went away during the spawn")


def check_spectator_and_window():
    print("spectator view, start_window")
    import carla
    from unittest import mock

    extent = V(2.4, 1.0, 0.75)
    for yaw in (0.0, 90.0, -135.0):
        tf = carla.Transform(carla.Location(10.0, 20.0, 1.0), carla.Rotation(yaw=yaw))
        chase = chase_view.follow_transform(tf, extent, 0)
        back = (chase.location.x - 10.0) * math.cos(math.radians(yaw)) + (chase.location.y - 20.0) * math.sin(math.radians(yaw))
        check(abs(back + 2.0 * 2.9) < 1e-6 and abs(chase.location.z - (1.0 + 2.0 * 1.25)) < 1e-6
              and math.hypot(chase.location.x - 10.0, chase.location.y - 20.0) - 5.8 < 1e-6, f"chase view at yaw {yaw}: {chase}")
        check(abs(chase.rotation.yaw - yaw) < 1e-6 and abs(chase.rotation.pitch - 8.0) < 1e-6, f"chase rotation at yaw {yaw}")
        top = chase_view.follow_transform(tf, extent, 2)
        check(abs(top.location.x - 10.0) < 1e-6 and abs(top.location.y - 20.0) < 1e-6 and abs(top.location.z - 26.0) < 1e-6
              and top.rotation.pitch == -90.0, f"top-down view at yaw {yaw}: {top}")

    class Spectator:
        def __init__(self, x):
            self.transform = carla.Transform(carla.Location(x, 0, 3), carla.Rotation())
            self.moves = 0

        def get_transform(self):
            return self.transform

        def set_transform(self, transform):
            self.transform, self.moves = transform, self.moves + 1

    class Snapshot:
        def __init__(self, states):
            self.states = states

        def find(self, actor_id):
            return self.states.get(actor_id)

    hero = Fake("vehicle.lincoln.mkz_2017", 0, 0)
    state = type("State", (), {"get_transform": lambda self: carla.Transform(carla.Location(0, 0, 0), carla.Rotation())})()

    class World:
        def __init__(self, spectator, states):
            self.spectator, self.states = spectator, states

        def get_spectator(self):
            return self.spectator

        def get_snapshot(self):
            return Snapshot(self.states)

        def spawn_actor(self, *args, **kwargs):
            check(False, "a sensor was spawned")

    world = World(Spectator(-6.0), {hero.id: state})
    view = chase_view.SpectatorView(world, hero, (320, 240))
    view.set_view(1)
    check(view.surface() is None and world.spectator.moves == 1 and world.spectator.transform.location.x > 0,
          f"the spectator follows the hero in the bonnet view: {world.spectator.transform}")
    gone = World(Spectator(-6.0), {})  # the hero is not in the snapshot any more: its handle would give the origin
    chase_view.SpectatorView(gone, hero, (320, 240)).surface()
    check(gone.spectator.moves == 0, "a hero that is gone must not move the spectator")
    far = World(Spectator(-500.0), {hero.id: state})
    chase_view.SpectatorView(far, hero, (320, 240)).surface()
    check(far.spectator.moves == 0, f"a move of more than {chase_view.SPECTATOR_MAX_MOVE} m must be skipped")
    view.close()

    with tempfile.TemporaryDirectory() as folder, mock.patch.object(runner.os, "name", "posix"), \
            mock.patch.object(runner.subprocess, "Popen") as popen, mock.patch.dict(os.environ, {"DISPLAY": ":0"}):
        log = Path(folder) / "chase_view.log"
        runner.start_window(["python3", "x.py"], folder, log)
        kwargs = popen.call_args[1]
        check(kwargs.get("start_new_session") is True and "creationflags" not in kwargs and log.exists()
              and kwargs.get("stderr") is runner.subprocess.STDOUT, f"start_window on Linux: {kwargs}")
        runner.start_window(["python3", "x.py"], folder)
        check(popen.call_args[1].get("stdout") is None, "without a log file the output is inherited")
    with mock.patch.object(runner.os, "name", "posix"), mock.patch.object(runner.subprocess, "Popen"), \
            mock.patch.dict(os.environ, {"DISPLAY": ""}), mock.patch("builtins.print") as printed:
        runner.start_window(["python3", "x.py"], ".")
        check(any("DISPLAY" in str(c.args[0]) for c in printed.call_args_list), "no warning without DISPLAY")


def check_scenario_runner_flags():
    print("ScenarioRunner v0.9.15 and manual_control.py flags")
    import ast
    import run_scenario
    from unittest import mock
    import manual_control_tm

    def options(script):
        tree = ast.parse(script.read_text(encoding="utf-8"))
        return {arg.value for node in ast.walk(tree) if isinstance(node, ast.Call)
                and getattr(node.func, "attr", None) == "add_argument"
                for arg in node.args if isinstance(arg, ast.Constant) and str(arg.value).startswith("-")}

    sr = options(runner.SCENARIO_RUNNER_ROOT / "scenario_runner.py")
    check("--trafficManagerPort" in sr and "--frameRate" not in sr, f"not ScenarioRunner v0.9.15's parser: {sorted(sr)}")
    args = run_scenario.scenario_runner_arguments("V_1", "s.py", "s.xml", "10.0.0.5", 2100, 60.0, 8100, "out", True,
                                                  record="rec", debug=True)
    flags = {a for a in map(str, args) if a.startswith("--")}
    check(flags <= sr, f"flags ScenarioRunner v0.9.15 does not know: {sorted(flags - sr)}")
    check("--sync" in flags and "--trafficManagerPort" in flags, f"flags {sorted(flags)}")
    mc = options(runner.manual_control_script())
    flags = {a for a in runner.manual_control_arguments("10.0.0.5", 2100) if a.startswith("--")}
    check(flags <= mc, f"flags manual_control.py does not know: {sorted(flags - mc)}")

    # the driving window's set_autopilot goes to our Traffic Manager port, an explicit one still wins
    calls = []
    with mock.patch.object(manual_control_tm.carla.Vehicle, "set_autopilot", lambda *a: calls.append(a)):
        manual_control_tm.pin_autopilot_port(8100)
        manual_control_tm.carla.Vehicle.set_autopilot("car", False)
        manual_control_tm.carla.Vehicle.set_autopilot("car", True, 9000)
    check(calls == [("car", False, 8100), ("car", True, 9000)], f"set_autopilot calls {calls}")
    with mock.patch.object(runner, "start_window") as start:
        runner.start_manual_control("127.0.0.1", 2000, None, 8100)
    cmd = start.call_args[0][0]
    check(cmd[1].endswith("manual_control_tm.py") and cmd[2:4] == ["--tm-port", "8100"] and "--keep_ego_vehicle" in cmd,
          f"driving window command {cmd}")


def main():
    for part in (check_nearest_ahead, check_ttc, check_hud_lines, check_args, check_panel, check_control_files,
                 check_run_loop, check_camera_never_orphaned, check_spectator_and_window, check_scenario_runner_flags):
        part()
    if FAILURES:
        print(f"\n{len(FAILURES)} check(s) FAILED")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
