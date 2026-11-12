# Runs inside the editor through cmd.py (vm.sh audit): writes every mesh in every tile of a generated map, with its
# place and size, to /work/logs/@NAME@_meshes.jsonl, for check_level.py. Changes nothing and saves nothing.
# vm.sh puts _Tile_ (every tile) or one tile like _Tile_2_2 in place of @TILES@.
import json

OUT = "/work/logs/@NAME@_meshes.jsonl"
import os
DISK = "/work/carla/Unreal/CarlaUE4/Content/CustomMaps/@NAME@"  # list_assets leaves out the tile maps
tiles = sorted("/Game/CustomMaps/@NAME@/" + f[:-5] for f in os.listdir(DISK)
               if f.endswith(".umap") and ("_Tile_" in f if "@TILES@" == "_Tile_" else f[:-5].endswith("@TILES@")))
with open(OUT, "w") as out:
    for tile in tiles:
        unreal.EditorLevelLibrary.load_level(tile)
        # a tile keeps its meshes relative to its south-west corner; tiles are 500 m, X counts east and Y north
        tx, ty = (int(v) * 500 for v in tile.rsplit("_Tile_", 1)[1].split("_"))
        n = 0
        for actor in unreal.EditorLevelLibrary.get_all_level_actors():
            label = actor.get_actor_label()
            if label.startswith(("TreeSpawnPosition", "MiscSpawnPosition")):  # the tool's spots, before the blueprint
                p = actor.get_actor_location()
                out.write(json.dumps({"tile": tile.rsplit("/", 1)[1], "actor": label, "mesh": "spot",
                                      "x": p.x / 100 + tx, "y": p.y / 100 - ty, "z": p.z / 100, "box": [0] * 6}) + "\n")
            for comp in actor.get_components_by_class(unreal.StaticMeshComponent):
                mesh = comp.static_mesh
                if mesh is None:
                    continue
                if isinstance(comp, unreal.InstancedStaticMeshComponent):
                    places = [comp.get_instance_transform(i, True).translation for i in range(comp.get_instance_count())]
                else:
                    places = [comp.get_world_location()]
                origin, extent = actor.get_actor_bounds(False)
                for p in places:
                    out.write(json.dumps({"tile": tile.rsplit("/", 1)[1], "actor": actor.get_actor_label(),
                                          "mesh": mesh.get_path_name(), "x": p.x / 100 + tx, "y": p.y / 100 - ty, "z": p.z / 100,
                                          "box": [origin.x / 100 + tx, origin.y / 100 - ty, origin.z / 100,
                                                  extent.x / 100, extent.y / 100, extent.z / 100]}) + "\n")
                    n += 1
        unreal.log("audit: {} {} meshes".format(tile, n))
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("audit: finished @RUN@, {} tiles".format(len(tiles)))
