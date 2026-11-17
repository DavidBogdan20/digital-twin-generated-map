"""Publishes the hero car's state as JSON lines on 127.0.0.1:2100, for a client at the far end of an SSH tunnel.

usage: publish.py [rate Hz, default 10] [port, default 2100]   in the container on the VM, with CARLA running
A client forwards only this port (ssh -L 2100:localhost:2100) and reads one JSON object per line: the fields of
tunnel_check.CarlaReader.read() plus wall_time, or {"status": "no ego" | "no ticks" | "map changed"}. Reading CARLA
directly through the tunnel sends every client every frame's full snapshot (7.6 MB/s measured, 2026-11-13); this sends
about 3 KB/s. Read-only: never ticks, spawns or changes anything. Survives map loads, scenario restarts and clients
coming and going; waits for CARLA if it isn't up yet.
"""
import json
import socket
import sys
import threading
import time

from tunnel_check import CarlaReader

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 2100
clients, lock = [], threading.Lock()


def accept(server):
    while True:
        conn, addr = server.accept()
        conn.settimeout(2.0)  # a stalled client is dropped instead of holding up the others
        with lock:
            clients.append(conn)
        print("client connected", addr, flush=True)


def broadcast(line):
    with lock:
        for conn in clients[:]:
            try:
                conn.sendall(line)
            except OSError:
                clients.remove(conn)
                conn.close()
                print("client gone", flush=True)


def main():
    server = socket.create_server(("127.0.0.1", PORT))  # localhost only: reachable through the SSH tunnel, nowhere else
    threading.Thread(target=accept, args=(server,), daemon=True).start()
    print("publishing on 127.0.0.1:%d at %.0f Hz" % (PORT, RATE), flush=True)
    reader, ticks_seen = None, time.monotonic()
    while True:
        t = time.monotonic()
        try:
            if reader is None:
                reader = CarlaReader()
                print("connected to CARLA, map", reader.map.name, flush=True)
            state = reader.read()
            if state != "no ticks":
                ticks_seen = t
            elif t - ticks_seen > 10.0:  # a restarted CARLA never ticks for the old connection: connect again
                reader, ticks_seen = None, t
        except RuntimeError as e:  # CARLA down or restarting
            print("CARLA:", e, flush=True)
            reader, state = None, "no CARLA"
            time.sleep(5.0)
        msg = state if isinstance(state, dict) else {"status": state}
        if isinstance(state, dict):
            msg["lights"] = int(state["lights"])  # carla.VehicleLightState bit flags
        msg["wall_time"] = time.time()
        broadcast((json.dumps(msg) + "\n").encode())
        time.sleep(max(0.0, 1.0 / RATE - (time.monotonic() - t)))


if __name__ == "__main__":
    main()
