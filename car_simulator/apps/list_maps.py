#!/usr/bin/env python
"""
List the maps installed on the running CARLA server and optionally load one.

    python apps/list_maps.py
    python apps/list_maps.py --load Town12
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from carsim.sim import available_maps, connect, ensure_map, short_map_name  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--load", metavar="TOWN", help="load this map, e.g. Town12")
    args = parser.parse_args()

    client = connect(args.host, args.port)
    print(f"Server {client.get_server_version()}, client {client.get_client_version()}")

    if args.load:
        ensure_map(client, args.load)

    current = short_map_name(client.get_world().get_map().name)
    print(f"\nInstalled maps (* = loaded, _Opt = layered version):")
    for full in available_maps(client):
        name = short_map_name(full)
        print(f"  {'*' if name == current else ' '} {name:<16} {full}")


if __name__ == "__main__":
    main()
