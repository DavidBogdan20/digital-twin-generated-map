# Runs inside the editor through cmd.py (vm.sh prune): removes the trees and props check_level.py rejected
# (/work/logs/@NAME@_bad.json) from the tiles and saves them. Most are the blueprint's own scattered vegetation
# (instances of BP_InstancedMesh actors), which the tool patch can't reach; the rest are whole actors.
import collections
import json

TILE = 500  # m; a tile keeps its meshes relative to its south-west corner (X counts east, Y north)
bad = collections.defaultdict(list)
for r in json.load(open("/work/logs/@NAME@_bad.json")):
    tx, ty = (int(v) * TILE for v in r["tile"].rsplit("_Tile_", 1)[1].split("_"))
    bad[r["tile"]].append((r["actor"], r["x"] - tx, r["y"] + ty))  # back to the tile's own coordinates


def near(p, x, y):
    return abs(p.x / 100 - x) < 0.05 and abs(p.y / 100 - y) < 0.05


removed = 0
for tile, items in sorted(bad.items()):
    unreal.EditorLevelLibrary.load_level("/Game/CustomMaps/@NAME@/" + tile)
    labels = {a for a, _, _ in items}
    n = 0
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        if actor.get_actor_label() not in labels:
            continue
        mine = [(x, y) for a, x, y in items if a == actor.get_actor_label()]
        for comp in actor.get_components_by_class(unreal.StaticMeshComponent):
            if isinstance(comp, unreal.InstancedStaticMeshComponent):
                i = comp.get_instance_count() - 1
                while i >= 0:  # from the end: removing one never moves an index still to be checked
                    if any(near(comp.get_instance_transform(i, True).translation, x, y) for x, y in mine):
                        comp.remove_instance(i)
                        n += 1
                    i -= 1
            elif any(near(comp.get_world_location(), x, y) for x, y in mine):
                unreal.EditorLevelLibrary.destroy_actor(actor)
                n += 1
                break
    unreal.EditorLevelLibrary.save_current_level()
    unreal.log("prune: {} removed {} of {}".format(tile, n, len(items)))
    removed += n
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("prune: finished @RUN@, removed {}".format(removed))
