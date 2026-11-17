"""Helpers for talking to a running CARLA server."""

import math
import time

import carla

# Large maps (Town12, the Digital Twin maps such as WolfsburgHighway): load_world resets the streaming distances to
# CARLA's defaults (tile_stream_distance 3000, actor_active_distance 2000), and streaming that much at once crashed a
# 0.9.15 server on a 2 x 2 km generated map. ensure_map sets these after every load (and whenever they differ). Small
# maps ignore them.
TILE_STREAM_DISTANCE = 600.0  # [m] tiles load within this distance of the hero (else the spectator)
ACTOR_ACTIVE_DISTANCE = 450.0  # [m] actors farther from the hero are put to sleep
SPECTATOR_STEP = 100.0  # [m] the spectator moves at most this far at once: big jumps stream many tiles at once (Vulkan crash)
SPECTATOR_STEP_WAIT = 1.0  # [s] the server runs this long after each step
TRAFFIC_MANAGER_PORT = 8100  # ScenarioRunner's Traffic Manager; 8000 is taken by a web server on the VM
# Maps small enough to keep every tile loaded (stream, active [m]): streaming tiles in and out as the car moved crashed
# 0.9.15 on WolfsburgHighway (Vulkan, FVulkanResourceMultiBuffer, 2026-11-10); its 16 tiles fit in about 7 GB of video
# memory, and with nothing streaming no actor or sensor is ever put to sleep.
KEEP_LOADED = {"WolfsburgHighway": (3000.0, 3000.0)}


def streaming_for(town, tile_stream_distance=None, actor_active_distance=None):
    """The distances to use on `town`: what the caller asked for, else KEEP_LOADED's, else the module defaults."""
    if tile_stream_distance is None and actor_active_distance is None:
        return KEEP_LOADED.get(town, (TILE_STREAM_DISTANCE, ACTOR_ACTIVE_DISTANCE))
    return (TILE_STREAM_DISTANCE if tile_stream_distance is None else tile_stream_distance,
            ACTOR_ACTIVE_DISTANCE if actor_active_distance is None else actor_active_distance)


assert streaming_for("WolfsburgHighway") == (3000.0, 3000.0)
assert streaming_for("WolfsburgHighway", 350.0, 300.0) == (350.0, 300.0)
assert streaming_for("WolfsburgHighway", 600.0, 450.0) == (600.0, 450.0)  # explicit, even when equal to the defaults
assert streaming_for("Town12") == (TILE_STREAM_DISTANCE, ACTOR_ACTIVE_DISTANCE)


def connect(host="127.0.0.1", port=2000, timeout=30.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def short_map_name(full_name):
    """'/Game/Carla/Maps/Town12/Town12' -> 'Town12'."""
    return full_name.split("/")[-1]


def available_maps(client):
    return sorted(client.get_available_maps(), key=short_map_name)


def ensure_async(world):
    """
    Switch the server back to asynchronous mode.

    A ScenarioRunner run that crashes can leave the server in synchronous
    mode, which freezes the simulation until a client ticks it. A fixed time
    step is removed too, also on a server that is already asynchronous: with it
    the simulation advances that step per frame, so it does not run in real time.
    """
    settings = world.get_settings()
    changed = False
    if settings.synchronous_mode:
        print("Server was left in synchronous mode; switching it back to asynchronous.")
        settings.synchronous_mode = False
        changed = True
    if settings.fixed_delta_seconds:  # None or 0.0 when not set
        if not changed:
            print(f"Server has a fixed time step ({settings.fixed_delta_seconds:.4f} s) in asynchronous mode, "
                  "so it may not run in real time; removing it.")
        settings.fixed_delta_seconds = None
        changed = True
    if changed:
        world.apply_settings(settings)


def apply_streaming(world, tile_stream_distance=TILE_STREAM_DISTANCE, actor_active_distance=ACTOR_ACTIVE_DISTANCE):
    """Set the large-map streaming distances [m] when they differ (a no-op on small maps). Lowering is safe; raising
    them during a run made the server stream many tiles at once and crash."""
    settings = world.get_settings()
    if (abs(settings.tile_stream_distance - tile_stream_distance) < 1.0
            and abs(settings.actor_active_distance - actor_active_distance) < 1.0):
        return
    print(f"Large-map streaming: tile_stream_distance {settings.tile_stream_distance:.0f} -> {tile_stream_distance:.0f} m, "
          f"actor_active_distance {settings.actor_active_distance:.0f} -> {actor_active_distance:.0f} m")
    settings.tile_stream_distance = tile_stream_distance
    settings.actor_active_distance = actor_active_distance
    world.apply_settings(settings)


def ensure_map(client, town, load_timeout=600.0, timeout=30.0, tile_stream_distance=None,
               actor_active_distance=None):
    """
    Load `town` on the server unless it is already the active map, and set the large-map streaming distances (see
    apply_streaming; load_world resets them).
    """
    tile_stream_distance, actor_active_distance = streaming_for(town, tile_stream_distance, actor_active_distance)
    world = client.get_world()
    if short_map_name(world.get_map().name) == town:
        apply_streaming(world, tile_stream_distance, actor_active_distance)
        return world

    names = [short_map_name(m) for m in client.get_available_maps()]
    if town not in names:
        raise RuntimeError(
            f"Map '{town}' is not installed on the server. Installed maps: {', '.join(sorted(names))}.\n"
            "Town12/Town13 come with AdditionalMaps_0.9.16.zip (extract it into the CARLA root folder).")

    print(f"Loading {town} (large maps can take several minutes)...")
    client.set_timeout(load_timeout)
    try:
        world = client.load_world(town)
    finally:
        client.set_timeout(timeout)
    apply_streaming(world, tile_stream_distance, actor_active_distance)
    return world


def focus_spectator(world, location, height=40.0, settle_seconds=3.0, yaw=0.0, behind=0.0, pitch=-80.0):
    """
    Put the spectator camera above `location` so you can watch the scenario,
    `behind` metres back along `yaw` and looking `pitch` degrees down (behind=12, height=6, pitch=-20: a chase view
    of a car starting at `location`), and let the server run a few seconds to render that area. (On large maps
    the tiles are streamed around the hero vehicle, or around the spectator
    while there is none, since WorldSettings.spectator_as_ego defaults to
    True.) A fixed wait is not a guarantee that a tile has loaded: use
    wait_for_ground() before spawning anything there. A far target is approached in steps of SPECTATOR_STEP m with
    SPECTATOR_STEP_WAIT s between them: on a 0.9.15 large map a jump of a few hundred metres streamed so many meshes
    at once that the server crashed.
    """
    spectator = world.get_spectator()
    back_x, back_y = math.cos(math.radians(yaw)) * behind, math.sin(math.radians(yaw)) * behind
    goal = carla.Location(location.x - back_x, location.y - back_y, location.z + height)
    rotation = carla.Rotation(pitch=pitch, yaw=yaw)  # not -90: straight down is gimbal lock and mouse look gets rolled/inverted
    for point in spectator_steps(spectator.get_transform().location, goal):
        spectator.set_transform(carla.Transform(point, rotation))
        end = time.time() + SPECTATOR_STEP_WAIT
        while time.time() < end:
            world.wait_for_tick()
    spectator.set_transform(carla.Transform(goal, rotation))

    end = time.time() + settle_seconds
    while time.time() < end:
        world.wait_for_tick()


def spectator_steps(start, goal, step=SPECTATOR_STEP):
    """The points (carla.Location) a move from `start` to `goal` stops at on its way, at most `step` m apart; none for a short move."""
    distance = start.distance(goal)
    count = int(math.ceil(distance / step)) - 1
    return [carla.Location(start.x + (goal.x - start.x) * i / (count + 1), start.y + (goal.y - start.y) * i / (count + 1),
                           start.z + (goal.z - start.z) * i / (count + 1)) for i in range(1, count + 1)]


# Semantic labels of the road surface (carla.CityObjectLabel): road, sidewalk, and the
# ground / terrain meshes around it. Trees, poles, signs and bridge decks are not in it.
GROUND_LABELS = (
    carla.CityObjectLabel.Roads,
    carla.CityObjectLabel.Sidewalks,
    carla.CityObjectLabel.Ground,
    carla.CityObjectLabel.Terrain,
)


def wait_for_ground(world, surfaces, labels=GROUND_LABELS, tolerance=1.5, above=2.0, below=3.0, timeout=60.0,
                    no_hit=15.0, settle=10.0, generated=None):
    """
    Wait until the large-map tile around each of `surfaces` has loaded, i.e.
    until its road surface is there to stand on. Actors spawned before that
    fall through the world.

    `surfaces` is a carla.Location, or a list of them, at the *expected
    surface height* (an ego start z in the XML is the road z + about 0.3 m, a
    pedestrian spawn z is about 1 m above the ground: subtract that). For each
    one, World.ground_projection is asked for the first surface from `above` m
    over it down to `below` m under it. A tile counts as loaded once that hit
    has one of `labels` and lies within `tolerance` m of the expected height.
    (World.cast_ray is not used: it returned no hits on Town12 where
    ground_projection found the road.)
    Generated maps (the Digital Twin's, e.g. WolfsburgHighway) may have no surface the trace sees, or label it
    otherwise: after `no_hit` s a hit within `tolerance` with any label is accepted (its label is printed), and when the
    trace has hit nothing at all at the surfaces still waiting by then, the tiles get `settle` s more and those count
    as loaded (None in the result) without proof. Only on such a map (`generated`, by default
    carsim.road.deformed_map of the world's map): elsewhere no hit means an unloaded tile, so it keeps waiting.
    The check polls once per server tick, so it needs the server in
    asynchronous mode (see ensure_async). Returns the accepted
    carla.LabelledPoint (or None, see above) of each surface, in order; raises RuntimeError after
    `timeout` s.
    """
    if isinstance(surfaces, carla.Location):
        surfaces = [surfaces]
    if generated is None:
        from carsim.road import deformed_map  # here: carsim.road imports nothing from sim, but keep sim light
        generated = deformed_map(world.get_map().name)
    if not generated:
        no_hit = float("inf")  # the fallbacks below are for generated maps only
    labels = frozenset(labels)
    done = [None] * len(surfaces)
    seen = [[] for _ in surfaces]
    ever = [False] * len(surfaces)  # the trace hit anything at all there
    start = time.time()
    print("Waiting for the map to load at "
          + ", ".join(f"({s.x:.1f}, {s.y:.1f})" for s in surfaces) + "...")
    while True:
        for i, surface in enumerate(surfaces):
            if done[i] is not None:
                continue
            hit = world.ground_projection(carla.Location(surface.x, surface.y, surface.z + above), above + below)
            if hit is None:
                seen[i] = ["nothing"]
                continue
            ever[i] = True
            seen[i] = [f"{hit.label.name} z={hit.location.z:.1f}"]
            late = time.time() - start > no_hit
            if abs(hit.location.z - surface.z) <= tolerance and (hit.label in labels or late):
                if hit.label not in labels:
                    print(f"Accepting the surface at ({surface.x:.1f}, {surface.y:.1f}) labelled {hit.label.name}.")
                done[i] = hit
        if all(hit is not None for hit in done):
            print(f"Ground found after {time.time() - start:.1f} s.")
            return done
        if time.time() - start > no_hit and not any(ever[i] for i, hit in enumerate(done) if hit is None):
            missing = ", ".join(f"({surfaces[i].x:.1f}, {surfaces[i].y:.1f})" for i, hit in enumerate(done) if hit is None)
            print(f"WARNING: World.ground_projection hit nothing at {missing} in {no_hit:.0f} s (a generated map without "
                  f"traceable ground?): waiting {settle:.0f} s more for the tiles; the ground there is NOT verified, "
                  "so actors may spawn into an unloaded tile or at a wrong height.")
            end = time.time() + settle
            while time.time() < end:
                world.wait_for_tick()
            return done
        if time.time() - start > timeout:
            i = next(i for i, hit in enumerate(done) if hit is None)
            s = surfaces[i]
            raise RuntimeError(
                f"No road or ground surface found near z={s.z:.1f} at ({s.x:.1f}, {s.y:.1f}) after {timeout:.0f} s "
                f"(surface seen: {', '.join(seen[i]) or 'nothing'}). "
                "The map tile there did not load: check that the point is on the map and that the "
                "server is running and not paused or minimized.")
        world.wait_for_tick()


# role names ScenarioRunner gives the actors it spawns (the ego "hero", everything else "scenario")
SCENARIO_ROLES = ("hero", "scenario")
CHASE_VIEW_ROLE = "carsim_chase_view"  # role_name of apps/chase_view.py's camera


def destroy_sensors_on(world, actor, role=CHASE_VIEW_ROLE):
    """
    Destroy the sensors with role_name `role` attached to `actor` (the chase view's camera on the ego, from another
    process); returns how many. Before the ego goes: CARLA does not destroy sensors with their parent, and on a 0.9.15
    large map a sensor that is put to sleep (left behind, or far from the hero) crashed the server. A world or actor
    that is gone counts as none.
    """
    removed = 0
    try:
        sensors = [sensor for sensor in world.get_actors().filter("sensor.*")
                   if sensor.attributes.get("role_name") == role and sensor.parent is not None
                   and sensor.parent.id == actor.id]
    except (AttributeError, RuntimeError):
        return 0
    for sensor in sensors:
        try:
            removed += bool(sensor.destroy())
        except RuntimeError:
            pass
    return removed


def another_client_ticks(world, wait=1.0):
    """
    True when the world is synchronous and its frame still advances after `wait` seconds: another client (a running
    scenario) is stepping it. A synchronous world nobody steps (a killed run) does not advance and gives False.
    """
    if not world.get_settings().synchronous_mode:
        return False
    frame = world.get_snapshot().frame
    time.sleep(wait)
    return world.get_snapshot().frame != frame


def reset_world(client, traffic_manager_port=None, all_controllers=False):
    """
    Return the server to a normal state after a ScenarioRunner run: asynchronous
    mode and no ego or scenario actors left. ScenarioRunner does this itself on
    a clean exit, but when it is killed the server stays synchronous, so nothing
    advances unless a client ticks it, which freezes the CARLA window (camera
    and mouse included).

    AI walker controllers have no role name. The ones removed are those attached to a leftover walker and the
    orphaned ones (no parent any more, because the walker was destroyed first). `all_controllers=True` removes
    every AI walker controller on the server, e.g. those of a generate_traffic.py run too.

    Sensors attached to a leftover actor (manual_control.py's camera, collision, lane invasion, GNSS and IMU sensors
    on the ego, when its window did not get to remove them) are removed with it: CARLA does not destroy them along
    with their parent. Orphaned sensors (no parent) are left alone, since they may belong to another client, except
    with `all_controllers=True`, the opt-in for everything that does not clearly belong to a scenario, and except the
    chase view's camera (role CHASE_VIEW_ROLE), which a viewer killed before its cleanup leaves behind.

    `traffic_manager_port=None` (the default) leaves the Traffic Manager alone. Always use it before starting
    ScenarioRunner: client.get_trafficmanager() creates the Traffic Manager in THIS process when none runs, so
    ScenarioRunner would attach to it as a client on the same port and nobody steps it in synchronous mode (all
    autopilot traffic stands still). It is also the default after a run: ScenarioRunner's Traffic Manager ends with its
    process, and a new one builds its road map first (over 7 minutes on a large generated map). With a port, a
    Traffic Manager still serving there is switched back to asynchronous.
    """
    world = client.get_world()
    ensure_async(world)
    if traffic_manager_port is not None:
        try:
            client.get_trafficmanager(traffic_manager_port).set_synchronous_mode(False)
        except RuntimeError:
            pass  # no traffic manager running
    actors = world.get_actors()
    leftovers = [a for a in actors if a.attributes.get("role_name") in SCENARIO_ROLES]
    leftover_ids = {a.id for a in leftovers}
    controllers = [a for a in actors.filter("controller.ai.walker")
                   if all_controllers or a.parent is None or a.parent.id in leftover_ids]
    # Collected now, while the parents still exist: once they are destroyed `parent` is None
    sensors = [a for a in actors.filter("sensor.*")
               if (a.parent is None and (all_controllers or a.attributes.get("role_name") == CHASE_VIEW_ROLE))
               or (a.parent is not None and a.parent.id in leftover_ids)]
    removed = 0
    removed_sensors = 0
    for sensor in sensors:
        try:
            if sensor.is_listening():
                sensor.stop()
        except RuntimeError:
            pass
        try:
            if sensor.destroy():
                removed_sensors += 1
        except RuntimeError:
            pass
    for controller in controllers:
        try:
            controller.stop()
        except RuntimeError:
            pass
        try:
            if controller.destroy():
                removed += 1
        except RuntimeError:
            pass
    for actor in leftovers:
        try:
            actor.destroy()
        except RuntimeError:
            pass
    if leftovers:
        print(f"Removed {len(leftovers)} leftover scenario actor(s).")
    if removed:
        print(f"Removed {removed} AI walker controller(s).")
    if removed_sensors:
        print(f"Removed {removed_sensors} sensor(s) left by the scenario actors.")
