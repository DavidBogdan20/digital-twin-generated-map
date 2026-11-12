#!/usr/bin/env python
"""
Replay a run recorded with `run_scenario.py --record` on the running CARLA server.

    python apps/replay.py                              # newest outputs/*/ folder that has a recording
    python apps/replay.py outputs/20261029_222630_PedestrianCrossing_Town10HD_1
    python apps/replay.py path/to/recording.log --start 5 --duration 20 --speed 0.5
    python apps/replay.py --no-follow                  # free camera instead of following the ego

The recording is a CARLA recorder .log file; the server reads it, so a path on this
machine only works with a server on this machine. If the recorded map is not the loaded
one it is loaded first (large maps can take minutes). The server is switched to
asynchronous mode. Playback runs on the server and this script returns once it has
started; the camera follows the ego (the actor with role_name hero) unless --no-follow. The playback
speed is set on every replay (the server would otherwise keep the one of the last replay).
"""

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from carsim.paths import OUTPUTS_DIR  # noqa: E402
from carsim.sim import connect, ensure_async, ensure_map, short_map_name  # noqa: E402

# ScenarioRunner's own console log in a run folder, not a CARLA recording
NOT_A_RECORDING = "scenario_runner.log"


def recording_in(folder):
    """The CARLA recorder log of a run folder (as noted in run.json, else the newest other .log), or None."""
    try:
        recorded = json.loads((folder / "run.json").read_text(encoding="utf-8")).get("record")
    except (OSError, ValueError, AttributeError):
        recorded = None
    if recorded and Path(recorded).is_file():
        return Path(recorded)
    logs = [p for p in folder.glob("*.log") if p.name != NOT_A_RECORDING]
    return max(logs, key=lambda p: p.stat().st_mtime) if logs else None


def find_recording(arg):
    """Resolve the command-line argument (None, a run folder or a .log file) to a recording file."""
    if arg is None:
        folders = sorted((d for d in OUTPUTS_DIR.glob("*") if d.is_dir()), key=lambda d: d.name, reverse=True)
        for folder in folders:
            found = recording_in(folder)
            if found:
                return found
        sys.exit(f"No recording found in {OUTPUTS_DIR}.\n"
                 "Record a run first:  python -m uv run python apps\\run_scenario.py pedestrian_crossing --record")

    path = Path(arg)
    if not path.exists() and not path.is_absolute() and (OUTPUTS_DIR / path).exists():
        path = OUTPUTS_DIR / path  # a bare run folder name
    if not path.exists():
        sys.exit(f"{arg} does not exist.")
    if path.is_dir():
        found = recording_in(path)
        if found is None:
            sys.exit(f"No recording (.log written by --record) in {path}. Was the run started with --record?")
        return found
    return path


def recorded_map_and_ego(info):
    """
    Map name and ego actor id from the text of Client.show_recorder_file_info(file, False), which looks like

        Version: 1
        Map: Town10HD_Opt
        Date: 10/29/26 22:26:42

        Frame 1 at 0 seconds
         Create 2190: spectator (6) at (-260, -200, 382.001)
         Create 2192: vehicle.lincoln.mkz_2017 (2) at (28.7, -67.9, 0.5)
          number_of_wheels = 4
          role_name = hero

    The ego is the actor with role_name hero, else the first vehicle.lincoln created, else None.
    """
    map_match = re.search(r"^\s*Map:\s*(\S+)", info, re.M)
    hero = lincoln = None
    current = None
    for line in info.splitlines():
        created = re.match(r"\s*Create\s+(\d+):\s*(\S+)", line)
        if created:
            current = int(created.group(1))
            if lincoln is None and created.group(2).startswith("vehicle.lincoln"):
                lincoln = current
            continue
        role = re.match(r"\s+role_name\s*=\s*(.*?)\s*$", line)
        if role and current is not None and role.group(1) == "hero" and hero is None:
            hero = current
    return (map_match.group(1) if map_match else None), (hero if hero is not None else lincoln)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", nargs="?",
                        help="run folder or .log file (default: newest outputs/*/ folder with a recording)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="CARLA client timeout [s]; loading the recorded map can take a while")
    parser.add_argument("--start", type=float, default=0.0,
                        help="start time in the recording [s]; negative counts from the end (use --start=-10)")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to replay (default 0: to the end)")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed factor (2 = double, 0.5 = half)")
    parser.add_argument("--no-follow", action="store_true", help="free camera: do not follow the ego")
    args = parser.parse_args()

    path = find_recording(args.recording).resolve()
    print(f"Recording: {path}")

    try:
        client = connect(args.host, args.port, args.timeout)
        info = client.show_recorder_file_info(str(path), False)
        if not info or re.match(r"\s*File .*not found", info):
            sys.exit(f"The server could not read the recording ({info.strip() or 'empty answer'}).")
        map_name, ego_id = recorded_map_and_ego(info)

        # replay_file switches to the recorded map by itself; load it here first so a map that is missing
        # on the server, or slow to load, is reported properly
        if map_name:
            world = ensure_map(client, short_map_name(map_name), timeout=args.timeout)
        else:
            print("Could not read the map name from the recording; leaving the loaded map.")
            world = client.get_world()
        ensure_async(world)

        follow_id = 0 if args.no_follow or ego_id is None else ego_id
        if not args.no_follow and ego_id is None:
            print("No ego (role_name hero) found in the recording; using the free camera.")
        print(f"Replaying from {args.start:g} s"
              + (f" for {args.duration:g} s" if args.duration > 0 else " to the end")
              + (f", following actor {follow_id}" if follow_id else ", free camera") + ".")
        # The replayer keeps these settings on the server between replays, and reads them when the replay starts,
        # so set all of them every time (a --speed 2 of an earlier replay must not stay), before replay_file, the
        # way CARLA's own start_replaying.py does: play the hero as recorded, and leave the recorded spectator
        # alone (that script moves the spectator only with --move-spectator; the follow camera needs no recorded one)
        client.set_replayer_time_factor(args.speed)
        client.set_replayer_ignore_hero(False)
        client.set_replayer_ignore_spectator(True)
        answer = client.replay_file(str(path), args.start, args.duration, follow_id)
        if answer:
            print(answer.strip())
    except RuntimeError as e:  # includes CARLA's timeout when no server is running
        sys.exit(f"{e}")

    print("Playback is running on the server. Run apps/reset_world.py afterwards to clear leftovers if it was interrupted.")


if __name__ == "__main__":
    main()
