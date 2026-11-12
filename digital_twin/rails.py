# Runs inside the editor through cmd.py (vm.sh rails): places CARLA's highway guard rail along the polylines of
# /work/dt/@NAME@_rails.json (rails_plan.py) in the tiles of @NAME@, piece after piece, and saves the tiles. The mesh's
# own bounds give its length and orientation. Nothing may stay selected (selecting crashes this editor), so the
# selection is cleared after every spawn, in the same frame.
import json
import math
import os

MESH = "/Game/Carla/Static/GuardRail/SM_Guardrail_02"
DISK = "/work/carla/Unreal/CarlaUE4/Content/CustomMaps/@NAME@"
data = json.load(open("/work/dt/@NAME@_rails.json"))
TILE = data["tile"]
mesh = unreal.load_asset(MESH)
bounds = mesh.get_bounds()
o, e = bounds.origin, bounds.box_extent
along_x = e.x >= e.y
length = 2 * (e.x if along_x else e.y) / 100.0  # m
start = ((o.x - e.x) if along_x else (o.y - e.y)) / 100.0  # where the mesh starts along its long axis, from its pivot
bottom = (o.z - e.z) / 100.0


def pieces(line):
    """(x, y, z, heading) of each piece's start along the polyline, `length` m apart."""
    out, carry = [], 0.0
    for (x0, y0, z0), (x1, y1, z1) in zip(line, line[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        t = carry
        while t < seg:
            f = t / seg
            out.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, z0 + (z1 - z0) * f, math.atan2(y1 - y0, x1 - x0)))
            t += length
        carry = t - seg
    return out


by_tile = {}
for line in data["lines"]:
    for x, y, z, h in pieces(line):
        by_tile.setdefault((int(x // TILE), int(-y // TILE)), []).append((x, y, z, h))
placed = 0
for (tx, ty), items in sorted(by_tile.items()):
    name = "@NAME@_Tile_%d_%d" % (tx, ty)
    if not os.path.exists(os.path.join(DISK, name + ".umap")):
        continue
    unreal.EditorLevelLibrary.load_level("/Game/CustomMaps/@NAME@/" + name)
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if actor.get_actor_label().startswith("GuardRail"):
            unreal.EditorLevelLibrary.destroy_actor(actor)  # a second run replaces the rails, not doubles them
    for i, (x, y, z, h) in enumerate(items):
        yaw = h if along_x else h - math.pi / 2  # the mesh's long axis along the road
        ux, uy = (math.cos(yaw), math.sin(yaw)) if along_x else (-math.sin(yaw), math.cos(yaw))
        lx, ly = x - TILE * tx - ux * start, y + TILE * ty - uy * start  # tile coordinates, mesh start on the point
        actor = unreal.EditorLevelLibrary.spawn_actor_from_object(
            mesh, unreal.Vector(lx * 100, ly * 100, (z - bottom) * 100), unreal.Rotator(0, 0, math.degrees(yaw)))
        unreal.EditorLevelLibrary.set_selected_level_actors([])
        actor.set_actor_label("GuardRail_%d" % i)
        placed += 1
    unreal.EditorLevelLibrary.save_current_level()
    unreal.log("rails: {} {} pieces".format(name, len(items)))
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("rails: finished @RUN@, {} pieces of {:.2f} m".format(placed, length))
