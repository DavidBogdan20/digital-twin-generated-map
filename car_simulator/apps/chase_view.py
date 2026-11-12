#!/usr/bin/env python
"""
A chase-view window for an autopilot run: a camera on the ego (the actor with role_name hero) in a pygame window, with a
HUD (speed, throttle/brake/steer, the nearest vehicle/walker/bicycle ahead with distance and time to collision,
simulation time, map). run_scenario.py starts it for autopilot runs; it can be run by hand too.

    python apps/chase_view.py
    python apps/chase_view.py --res 1920x1080 --wait 300
    python apps/chase_view.py --spectator

Keys (window focused): C cycles the view (chase, bonnet, top-down), H toggles the HUD, Esc closes the window.

It is a viewer only: it never applies control, never ticks the world and never changes the world settings (so it can
sit next to a ScenarioRunner that runs the server in synchronous mode). It waits for the hero to appear (--wait
seconds, then gives up with exit code 1) and closes itself when the hero is gone. Closing the window does not touch the
scenario. The camera is destroyed on every way out (ComposedScenario's ego_control block also destroys it before the
ego, see carsim.blocks.ego_control).

--spectator spawns no sensor at all: the server's spectator follows the ego instead (in the same views), so the picture
is in the CARLA window (streamed with Moonlight on the VM) and this window shows the HUD and the buttons on black. Use it
when a camera sensor crashes the server: CARLA 0.9.15's large-map manager can put a sensor to sleep, which crashed it.

With --control DIR (run_scenario.py passes it) the window is the control panel of the run instead: it has two buttons,
Restart (R) and Quit (Esc, or closing the window), which write restart.request / quit.request in DIR for run_scenario.py
(restart ends the run cleanly and starts it again, quit ends it and run_scenario.py exits). When the hero is gone the
window stays open with the last frame and a banner (the result and the metrics line, from DIR/result.json, which
run_scenario.py writes), and attaches to the next hero by itself. Still no control and no tick.
"""

import argparse
import json
import math
import signal
import sys
import threading
import time
from collections import namedtuple
from pathlib import Path

import numpy as np
import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import carla  # noqa: E402

from carsim.runner import QUIT_REQUEST, RESTART_REQUEST, read_result, write_request  # noqa: E402
from carsim.sim import CHASE_VIEW_ROLE, connect, short_map_name  # noqa: E402

LEGEND = "C view   H hud   Esc quit"
BUTTON_LABELS = {"restart": "Restart (R)", "quit": "Quit (Esc)"}
BUTTON_SIZE = (150, 34)
GREY = (160, 160, 160)
BANNER_COLORS = {"SUCCESS": (60, 170, 80), "FAILURE": (200, 60, 60)}  # anything else (and "ended"): grey
HERO_POLL = 0.5  # s between looks for a new hero while there is none
AHEAD_RANGE = 50.0  # m, from the front bumper
AHEAD_HALF_WIDTH = 1.25  # m, the corridor along the ego's heading is 2.5 m wide
SPECTATOR_MAX_MOVE = 50.0  # m: --spectator never moves the spectator farther at once (long jumps crashed 0.9.15)

# name, offset from the ego (x forward, y right, z up), pitch, attachment, offset in metres (False) or, like
# manual_control.py's cameras, in units of the bounding box half size + 0.5 m (True)
VIEWS = (
    ("chase", (-2.0, 0.0, 2.0), 8.0, carla.AttachmentType.SpringArm, True),
    ("bonnet", (0.8, 0.0, 1.3), 0.0, carla.AttachmentType.Rigid, True),
    ("top-down", (0.0, 0.0, 25.0), -90.0, carla.AttachmentType.Rigid, False),
)

Ahead = namedtuple("Ahead", "kind distance ttc")  # distance in m from the ego's front bumper, ttc in s or None


def actor_kind(actor):
    """'walker', 'bicycle', 'vehicle', or None for anything else. (Motorbikes count as bicycles: 2 wheels.)"""
    if actor.type_id.startswith("walker."):
        return "walker"
    if actor.type_id.startswith("vehicle."):
        return "bicycle" if actor.attributes.get("number_of_wheels") == "2" else "vehicle"
    return None


def nearest_ahead(ego, others, max_dist=AHEAD_RANGE, half_width=AHEAD_HALF_WIDTH):
    """
    The nearest vehicle/walker/bicycle of `others` in the corridor `half_width` m to each side of the ego's heading,
    up to `max_dist` m beyond its front bumper, as an Ahead, or None. The distance is bumper to the other's centre.
    TTC is that distance over the closing speed along the heading, None when the gap is not closing.
    """
    tf = ego.get_transform()
    yaw = math.radians(tf.rotation.yaw)
    hx, hy = math.cos(yaw), math.sin(yaw)
    ego_v = ego.get_velocity()
    ego_along = ego_v.x * hx + ego_v.y * hy
    front = ego.bounding_box.extent.x
    best = None
    for other in others:
        kind = actor_kind(other)
        if other.id == ego.id or kind is None:
            continue
        loc = other.get_transform().location
        dx, dy = loc.x - tf.location.x, loc.y - tf.location.y
        gap = dx * hx + dy * hy - front
        if not 0.0 <= gap <= max_dist or abs(-dx * hy + dy * hx) > half_width:
            continue
        if best is None or gap < best.distance:
            v = other.get_velocity()
            closing = ego_along - (v.x * hx + v.y * hy)
            best = Ahead(kind, gap, gap / closing if closing > 0.1 else None)
    return best


def hud_lines(speed_kmh, sim_time, map_name, view, ahead):
    """The HUD's text lines (the bars are drawn separately)."""
    if ahead is None:
        target = "Ahead: -   TTC -"
    else:
        ttc = "-" if ahead.ttc is None else f"{ahead.ttc:.1f} s"
        target = f"Ahead: {ahead.kind} {ahead.distance:.1f} m   TTC {ttc}"
    return [f"{speed_kmh:.0f} km/h", target, f"t {sim_time:.1f} s   {map_name}   view {view}", LEGEND]


def key_action(key):
    """'restart' for R, 'quit' for Esc, else None."""
    return {pygame.K_r: "restart", pygame.K_ESCAPE: "quit"}.get(key)


def button_rects(size, ended):
    """{'restart': (x, y, w, h), 'quit': ...}: small in the top-right corner during a run, centred under the banner once it ended."""
    (w, h), (bw, bh) = size, BUTTON_SIZE
    if ended:
        return {"restart": (w // 2 - bw - 10, h - bh - 40, bw, bh), "quit": (w // 2 + 10, h - bh - 40, bw, bh)}
    return {"restart": (w - 2 * bw - 24, 8, bw, bh), "quit": (w - bw - 8, 8, bw, bh)}


def hit_button(pos, rects):
    """The name of the button of `rects` that contains the point `pos`, or None."""
    for name, (x, y, w, h) in rects.items():
        if x <= pos[0] < x + w and y <= pos[1] < y + h:
            return name
    return None


KIND_NAMES = {"walker": "pedestrian", "bicycle": "cyclist", "vehicle": "vehicle"}  # the driver's kinds, in plain words


def banner_lines(result, requested=None, metrics=None):
    """
    (headline, detail lines, colour) of the end-of-run banner, in plain words. `result` is run_scenario's result.json
    (None: not written yet), `requested` what was asked ('restart' or None), `metrics` the run's metrics.json (None:
    read from the run folder named in `result`). A line whose numbers are missing is left out.
    """
    if requested == "restart":
        return "Restarting...", [], GREY
    result = result or {}
    if metrics is None and result.get("run_dir"):
        try:
            metrics = json.loads((Path(result["run_dir"]) / "metrics.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            metrics = None
    m = metrics or {}
    outcome = str(result.get("result") or "").upper()
    colour = BANNER_COLORS.get(outcome, GREY)
    seen, onset, emergency = m.get("first_visible") or {}, m.get("brake_onset") or {}, m.get("emergency") or {}
    impact = m.get("impact") or {}
    kind = KIND_NAMES.get(seen.get("kind"), "target")
    if impact.get("happened"):
        what = f"the {kind}" if impact.get("other_is_target") else "a " + str(impact.get("other_type") or "actor").split(".")[0]
        speed = impact.get("speed")
        headline = f"COLLISION: hit {what}" + ("" if speed is None else f" at {speed * 3.6:.0f} km/h")
        colour = BANNER_COLORS["FAILURE"]
    elif outcome == "SUCCESS":
        headline = "PASSED: no collision"
    elif outcome:
        headline = f"FAILED ({outcome.lower()})"
    else:
        headline = "Run ended"
    lines = []
    if m.get("test_speed") is not None:
        lines.append(f"Test speed: {m['test_speed'] * 3.6:.0f} km/h")
    if seen.get("distance") is not None:
        ttc = seen.get("ttc")
        lines.append(f"First saw the {kind} {seen['distance']:.1f} m ahead, "
                     + (f"{ttc:.1f} s before the car would have reached it" if ttc is not None else "not yet in the car's path"))
    if onset.get("t") is not None:
        react = f"{onset['t'] - seen['t']:.1f} s after seeing it" if seen.get("t") is not None else "for it"
        ttc = onset.get("ttc")
        lines.append(f"Started braking {react}" + ("" if ttc is None else f", {ttc:.1f} s before reaching it"))
        lines.append(f"Emergency braking at {emergency['speed'] * 3.6:.0f} km/h" if emergency.get("speed") is not None
                     else "No emergency braking: a normal stop")
    elif seen:
        lines.append("Did not need to brake")
    if impact.get("happened") and m.get("speed_reduction") is not None:
        lines.append(f"Slowed down by {m['speed_reduction'] * 3.6:.0f} km/h before the impact")
    elif m.get("min_distance") is not None:
        lines.append(f"Closest distance to the {kind}: {m['min_distance']:.1f} m")
    if not lines and result.get("summary"):
        lines.append(result["summary"])  # no metrics file: the console's one line
    return headline, lines, colour


def parse_res(text):
    try:
        width, height = (int(v) for v in text.lower().split("x"))
        if width > 0 and height > 0:
            return width, height
    except ValueError:
        pass
    raise argparse.ArgumentTypeError(f"{text!r} is not WIDTHxHEIGHT, e.g. 1280x720")


def follow_transform(transform, extent, index):
    """The spectator's carla.Transform for view `index` of VIEWS on a hero at `transform` with half sizes `extent`
    (the offset turned by the hero's yaw; no spring arm)."""
    _, offset, pitch, _, boxed = VIEWS[index]
    if boxed:
        offset = (offset[0] * (0.5 + extent.x), offset[1] * (0.5 + extent.y), offset[2] * (0.5 + extent.z))
    yaw = math.radians(transform.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    location = carla.Location(transform.location.x + offset[0] * c - offset[1] * s,
                              transform.location.y + offset[0] * s + offset[1] * c, transform.location.z + offset[2])
    return carla.Transform(location, carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw))


def find_hero(world):
    for vehicle in world.get_actors().filter("vehicle.*"):
        if vehicle.attributes.get("role_name") == "hero":
            return vehicle
    return None


def wait_for_hero(client, wait):
    """(world, hero) once a hero exists, or None after `wait` seconds. The world is fetched again each time: a map may be loaded meanwhile."""
    end = time.time() + wait
    print("Waiting for the hero vehicle...")
    while time.time() < end:
        try:
            world = client.get_world()
            hero = find_hero(world)
            if hero is not None:
                return world, hero
        except RuntimeError:
            pass  # the server is busy, e.g. loading a map
        time.sleep(0.5)
    return None


class Camera:
    """An RGB camera attached to the hero in one of VIEWS; the newest image is kept for the main thread."""

    def __init__(self, world, hero, size):
        self.world, self.hero = world, hero
        self.sensor = None
        self.image = None
        self.lock = threading.Lock()
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(size[0]))
        bp.set_attribute("image_size_y", str(size[1]))
        bp.set_attribute("gamma", "2.2")
        bp.set_attribute("role_name", CHASE_VIEW_ROLE)  # reset_world() removes a camera left behind by a killed viewer
        self.blueprint = bp

    def set_view(self, index):
        _, offset, pitch, attachment, boxed = VIEWS[index]
        self.close()
        if boxed:
            ext = self.hero.bounding_box.extent
            offset = (offset[0] * (0.5 + ext.x), offset[1] * (0.5 + ext.y), offset[2] * (0.5 + ext.z))
        tf = carla.Transform(carla.Location(*offset), carla.Rotation(pitch=pitch))
        self.sensor = self.world.spawn_actor(self.blueprint, tf, attach_to=self.hero, attachment_type=attachment)
        self.sensor.listen(self._store)
        try:
            self.hero.get_transform()  # asks the server now (a snapshot is as old as the last tick): is the hero still there?
        except RuntimeError:
            self.close()  # the hero was destroyed meanwhile: a camera spawned on it then could be left without a parent
            raise

    def _store(self, image):
        with self.lock:
            self.image = image

    def surface(self):
        """The newest image as a pygame surface, or None before the first one."""
        with self.lock:
            image, self.image = self.image, None
        if image is None:
            return None
        rgb = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)[:, :, 2::-1]
        return pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

    def close(self):
        sensor, self.sensor = self.sensor, None
        if sensor is not None:
            try:
                sensor.stop()
                sensor.destroy()
            except RuntimeError:
                pass  # already gone with the world


class SpectatorView:
    """--spectator: the Camera interface without a sensor. surface() moves the server's spectator to the view behind the
    hero and returns None (the picture is in the CARLA window). The hero's pose comes from the world snapshot, so a hero
    that is gone moves nothing (its handle would give a zero transform: a jump to the origin), and a move of more than
    SPECTATOR_MAX_MOVE from where the spectator is now is skipped (once noted)."""

    def __init__(self, world, hero, size):
        self.world, self.hero = world, hero
        self.index = 0
        self.skipped = False

    def set_view(self, index):
        self.index = index
        self.hero.get_transform()  # raises RuntimeError when the hero is gone, as Camera.set_view does

    def surface(self):
        state = self.world.get_snapshot().find(self.hero.id)
        if state is None:
            return None
        target = follow_transform(state.get_transform(), self.hero.bounding_box.extent, self.index)
        spectator = self.world.get_spectator()
        if spectator.get_transform().location.distance(target.location) > SPECTATOR_MAX_MOVE:
            if not self.skipped:
                print(f"The spectator is more than {SPECTATOR_MAX_MOVE:g} m from the ego's view: not moved.")
            self.skipped = True
            return None
        spectator.set_transform(target)
        return None

    def close(self):
        pass


def draw_hud(display, font, lines, control):
    panel = pygame.Surface((330, 24 * len(lines) + 60), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 120))
    display.blit(panel, (8, 8))
    for i, line in enumerate(lines):
        display.blit(font.render(line, True, (255, 255, 255)), (16, 14 + 22 * i))
    y = 14 + 22 * len(lines)
    for label, value, signed in (("thr", control.throttle, False), ("brk", control.brake, False),
                                 ("str", control.steer, True)):
        display.blit(font.render(label, True, (255, 255, 255)), (16, y))
        pygame.draw.rect(display, (255, 255, 255), (60, y + 2, 200, 12), 1)
        if signed:  # steer: -1..1 around the middle
            left, width = 160 + min(0, value) * 100, abs(value) * 100
        else:
            left, width = 60, value * 200
        pygame.draw.rect(display, (255, 255, 255), (left, y + 2, max(width, 0), 12))
        y += 16


def draw_buttons(display, font, rects, disabled=()):
    for name, (x, y, w, h) in rects.items():
        grey = name in disabled
        pygame.draw.rect(display, (70, 70, 70) if grey else (30, 30, 30), (x, y, w, h))
        pygame.draw.rect(display, GREY if grey else (255, 255, 255), (x, y, w, h), 1)
        label = font.render(BUTTON_LABELS[name], True, GREY if grey else (255, 255, 255))
        display.blit(label, (x + (w - label.get_width()) // 2, y + (h - label.get_height()) // 2))


def draw_banner(display, big_font, font, size, lines):
    headline, details, colour = lines
    height = 66 + 26 * len(details)
    top = size[1] - BUTTON_SIZE[1] - 50 - height
    panel = pygame.Surface((size[0] - 40, height), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 170))
    display.blit(panel, (20, top))
    rows = [(headline, big_font, colour, 16)] + [(text, font, (255, 255, 255), 62 + 26 * i) for i, text in enumerate(details)]
    for text, f, c, y in rows:
        label = f.render(text, True, c)
        display.blit(label, ((size[0] - label.get_width()) // 2, top + y))


def view(client, world, hero, size, control=None, spectator=False):
    """
    Show the window until it is closed or, without `control`, the hero is gone. With `control` (a folder, see the
    module docstring) the window outlives the hero: it keeps the last frame, shows the banner and attaches to the next hero.
    `spectator`: the spectator follows the hero instead of a camera (SpectatorView).
    """
    pygame.init()
    pygame.font.init()
    display = pygame.display.set_mode(size)
    pygame.display.set_caption("Chase view (spectator: the picture is in the CARLA window)" if spectator else "Chase view")
    font = pygame.font.Font(None, 22)
    big_font = pygame.font.Font(None, 44)
    clock = pygame.time.Clock()
    camera = (SpectatorView if spectator else Camera)(world, hero, size)
    index, show_hud = 0, True
    frame = None  # the last camera image, kept when the hero is gone
    requested = None  # 'restart' once asked (until the next hero), so a second press does nothing
    result, map_name, start, next_poll = None, "", 0.0, 0.0

    def attach(new_world, new_hero):
        """Put the camera on `new_hero`; False (hero is None) when it vanished meanwhile."""
        nonlocal world, hero, map_name, start, frame, requested, result
        world, hero = new_world, new_hero
        camera.world, camera.hero = world, hero
        try:
            camera.set_view(index)
            map_name = short_map_name(world.get_map().name)
            start = world.get_snapshot().timestamp.elapsed_seconds
        except RuntimeError:
            hero = None
            return False
        frame, requested, result = None, None, None
        return True

    # SIGTERM / Ctrl+Break must run the finally below (SIGINT already does); a hard kill cannot be caught
    for name in ("SIGTERM", "SIGBREAK"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), lambda *_: sys.exit(0))
    try:
        attach(world, hero)
        running = control is not None or hero is not None
        while running:
            clock.tick(30)
            rects = button_rects(size, hero is None)
            for event in pygame.event.get():
                action = None
                if event.type == pygame.QUIT:
                    action = "quit"
                elif event.type == pygame.KEYUP:
                    action = key_action(event.key)
                    if event.key == pygame.K_c and hero is not None:
                        index = (index + 1) % len(VIEWS)
                        try:
                            camera.set_view(index)
                        except RuntimeError:
                            pass  # the hero is gone: the check below notices
                    elif event.key == pygame.K_h:
                        show_hud = not show_hud
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1 and control is not None:
                    action = hit_button(event.pos, rects)
                if action == "quit":
                    if control is not None:
                        write_request(control, QUIT_REQUEST)
                    running = False
                elif action == "restart" and control is not None and requested is None:
                    write_request(control, RESTART_REQUEST)
                    requested = "restart"
            if not running:
                break
            if hero is not None:
                try:
                    snapshot = world.get_snapshot()
                    lost = not snapshot.has_actor(hero.id)  # world.get_actor() and the handle keep a destroyed actor
                    if not lost:
                        surface = camera.surface()
                        if surface is not None:
                            frame = surface
                        if frame is not None:
                            display.blit(frame, (0, 0))
                        else:
                            display.fill((0, 0, 0))
                        if show_hud:
                            v = hero.get_velocity()
                            others = [*world.get_actors().filter("vehicle.*"), *world.get_actors().filter("walker.pedestrian.*")]
                            lines = hud_lines(3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
                                              snapshot.timestamp.elapsed_seconds - start, map_name,
                                              VIEWS[index][0], nearest_ahead(hero, others))
                            draw_hud(display, font, lines, hero.get_control())
                except RuntimeError:
                    lost = True  # the hero (or the server) is gone
                if lost:
                    if control is None:
                        break
                    camera.close()
                    hero = None
                    continue
                if control is not None:
                    draw_buttons(display, font, rects, ("restart",) if requested else ())
            else:  # the run is over: the last frame, the banner, and a look for the next hero
                if frame is not None:
                    display.blit(frame, (0, 0))
                else:
                    display.fill((0, 0, 0))
                if time.time() >= next_poll:
                    next_poll = time.time() + HERO_POLL
                    result = read_result(control)
                    try:
                        new_world = client.get_world()
                        new_hero = find_hero(new_world)
                    except RuntimeError:
                        new_hero = None  # the server is busy
                    if new_hero is not None and attach(new_world, new_hero):
                        continue
                draw_banner(display, big_font, font, size, banner_lines(result, requested))
                draw_buttons(display, font, rects, ("restart",) if requested else ())
            pygame.display.flip()
    except RuntimeError as e:  # the hero went away while the camera was being (re)spawned, or the server did
        print(f"Chase view stopped: {e}")
    finally:
        camera.close()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--res", type=parse_res, default=(1280, 720), metavar="WxH", help="window size (default 1280x720)")
    parser.add_argument("--control", metavar="DIR", help="control folder of run_scenario.py: the window then has the Restart and "
                        "Quit buttons and stays open across runs (see the top of this file)")
    parser.add_argument("--wait", type=float, default=120.0, help="give up after this many seconds without a hero (default 120)")
    parser.add_argument("--spectator", action="store_true", help="move the server's spectator behind the ego instead of "
                        "attaching a camera sensor (the picture is then in the CARLA window)")
    args = parser.parse_args()

    client = connect(args.host, args.port)
    found = wait_for_hero(client, args.wait)
    if found is None:
        print(f"No hero vehicle appeared within {args.wait:.0f} s.")
        return 1
    view(client, *found, args.res, args.control, args.spectator)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
