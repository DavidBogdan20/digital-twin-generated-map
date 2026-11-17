"""Read-only check of what a client gets from CARLA 0.9.15, locally or at the far end of a tunnel.

usage: tunnel_check.py [host, default localhost] [seconds, default 30]
Prints the server version and map, then once a second: snapshots received in that second, frames missed, the round
trip of one request, and the hero's state as CarlaReader gives it (HERBIE_DATA_PLAN.md). Never ticks, spawns or
changes anything. Exit code 1 if no snapshot arrived or no single hero was seen.
"""
import sys
import time

import carla


class CarlaReader:
    """Read-only state of the single 'hero' car. Never ticks, spawns, or touches settings or the Traffic Manager."""

    def __init__(self, host="localhost", port=2000, role="hero"):
        self.client = carla.Client(host, port)
        self.client.set_timeout(30.0)  # the first get_map() downloads the 4 MB road file through the tunnel
        version = self.client.get_server_version()
        assert version.startswith("0.9.15"), version
        self.role, self.world, self.latest = role, None, None
        self._attach()

    def _on_tick(self, snapshot):  # CARLA's stream thread: store only
        self.latest = (snapshot, time.monotonic())

    def _attach(self):
        if self.world is not None:
            self.world.remove_on_tick(self.tick_id)
        self.world = self.client.get_world()
        self.map = self.world.get_map()  # lat/lon are then computed locally
        self.episode = self.world.get_snapshot().id
        self.ego, self.next_find, self.next_lights, self.lights = None, 0.0, 0.0, None
        self.tick_id = self.world.on_tick(self._on_tick)

    def read(self):
        """The ego's latest state as a dict, or a status string."""
        now = time.monotonic()
        if self.latest is None or now - self.latest[1] > 2.0:
            return "no ticks"
        s = self.latest[0]
        if s.id != self.episode:  # a new map was loaded
            self._attach()
            return "map changed"
        if self.ego is None or not s.has_actor(self.ego.id):
            self.ego = None
            if now < self.next_find:
                return "no ego"
            self.next_find = now + 1.0
            heroes = [a for a in self.world.get_actors().filter("vehicle.*")
                      if a.attributes.get("role_name") == self.role]
            if len(heroes) != 1:
                return "no ego"
            self.ego = heroes[0]
            self.max_steer = self.ego.get_physics_control().wheels[0].max_steer_angle
        if now >= self.next_lights:  # the only request in the loop, once a second
            self.lights, self.next_lights = self.ego.get_light_state(), now + 1.0
        a, c = s.find(self.ego.id), self.ego.get_control()
        tf, v = a.get_transform(), a.get_velocity()
        f, geo = tf.get_forward_vector(), self.map.transform_to_geolocation(tf.location)
        return dict(frame=s.frame, sim_time=s.timestamp.elapsed_seconds,
                    speed=v.x * f.x + v.y * f.y + v.z * f.z,  # m/s, negative in reverse
                    gear="R" if c.reverse else "D", steer_deg=c.steer * self.max_steer,
                    throttle=c.throttle, brake=c.brake, lights=self.lights,
                    lat=geo.latitude, lon=geo.longitude, alt=geo.altitude,
                    heading=(tf.rotation.yaw + 90.0) % 360.0, speed_limit=self.ego.get_speed_limit())


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    t = time.monotonic()
    r = CarlaReader(host)
    print("server %s, map %s, connected and map read in %.1f s" % (
        r.client.get_server_version(), r.map.name, time.monotonic() - t), flush=True)
    frames, seen_snapshot, seen_ego = [], False, False
    r.world.on_tick(lambda s: frames.append(s.frame))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        time.sleep(1.0)
        got = frames[:]
        del frames[:len(got)]
        missed = sum(b - a - 1 for a, b in zip(got, got[1:]) if b - a > 1)
        t = time.monotonic()
        r.client.get_server_version()
        rtt = (time.monotonic() - t) * 1000.0
        st = r.read()
        seen_snapshot |= bool(got)
        if isinstance(st, dict):
            seen_ego = True
            st = ("frame %d t %.2f speed %.1f km/h gear %s steer %.1f deg throttle %.2f brake %.2f lat %.6f lon %.6f "
                  "alt %.1f heading %.0f lights %s limit %.0f" % (
                      st["frame"], st["sim_time"], st["speed"] * 3.6, st["gear"], st["steer_deg"], st["throttle"],
                      st["brake"], st["lat"], st["lon"], st["alt"], st["heading"], st["lights"], st["speed_limit"]))
        print("%3d snapshots/s, %d frames missed, request %.0f ms: %s" % (len(got), missed, rtt, st), flush=True)
    sys.exit(0 if seen_snapshot and seen_ego else 1)


if __name__ == "__main__":
    main()
