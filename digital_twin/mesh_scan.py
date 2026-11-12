# Runs inside the editor through cmd.py (vm.sh meshscan): finds the generated road meshes whose triangles break
# Unreal's distance field build, which crashed the cook of WolfsburgHighway (2026-11-10: "Array index out of bounds:
# 362 from an array of size 362" in TkDOPNode::SplitTriangleList, building SM_DrivingLaneMesh84_X_1_Y_3).
# The cause: SplitTriangleList reads past the end of its list when more than 4 triangles share one centre (kDOP.h,
# float rounding puts every centre below their mean). A zero-width lane (prepare_dt.py unpave) gives 6 collapsed
# triangles at one point wherever two of its rows meet, so such meshes had groups of 7. A mesh is bad then, or when a
# vertex, normal or tangent is NaN or infinite, or when no triangle has any area.
# With @FIX@ = 1 it switches the distance field off for each bad mesh (DistanceFieldResolutionScale 0, which skips the
# build) and saves it; only distance field shadows and ambient occlusion of that lane strip go. Writes
# /work/logs/@NAME@_badmeshes.jsonl.
import collections
import json
import math
import os

DISK = "/work/carla/Unreal/CarlaUE4/Content/CustomMaps/@NAME@"
tiles = sorted("/Game/CustomMaps/@NAME@/" + f[:-5] for f in os.listdir(DISK) if f.endswith(".umap") and "_Tile_" in f)
MIN_AREA = 1.0  # cm²: a triangle smaller than this counts as none
SHARED = 5  # triangles with one centre that crash the build (more than 4)


def finite(vectors):
    return all(math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z) for v in vectors)


def area(a, b, c):
    u, v = b - a, c - a
    return 0.5 * u.cross(v).length()


def check(mesh):
    """(problem or None, triangles, triangles with area)"""
    tris = good = 0
    problem = None
    centres = collections.Counter()
    for section in range(mesh.get_num_sections(0)):
        verts, idx, normals, _, tangents = unreal.ProceduralMeshLibrary.get_section_from_static_mesh(mesh, 0, section)
        if not finite(verts) or not finite(normals) or not finite([t.tangent_x for t in tangents]):
            problem = "not finite"
        for i in range(0, len(idx) - 2, 3):
            a, b, c = verts[idx[i]], verts[idx[i + 1]], verts[idx[i + 2]]
            tris += 1
            if area(a, b, c) >= MIN_AREA:
                good += 1
            # 0.1 cm: centres that close may still round to one in Unreal's floats
            centres[(round((a.x + b.x + c.x) / 3, 1), round((a.y + b.y + c.y) / 3, 1), round((a.z + b.z + c.z) / 3, 1))] += 1
    if problem is None and good == 0:
        problem = "no area"
    if problem is None and centres and max(centres.values()) >= SHARED:
        problem = "{} triangles share a centre".format(max(centres.values()))
    return problem, tris, good


seen = set()
fixed = 0
with open("/work/logs/@NAME@_badmeshes.jsonl", "w") as out:
    for tile in tiles:
        unreal.EditorLevelLibrary.load_level(tile)
        for actor in unreal.EditorLevelLibrary.get_all_level_actors():
            for comp in actor.get_components_by_class(unreal.StaticMeshComponent):
                mesh = comp.static_mesh
                if mesh is None or isinstance(comp, unreal.InstancedStaticMeshComponent):
                    continue
                path = mesh.get_path_name()
                if "/CustomMaps/@NAME@/Static/" not in path or path in seen:
                    continue
                seen.add(path)
                problem, tris, good = check(mesh)
                if problem is None:
                    continue
                out.write(json.dumps({"tile": tile.rsplit("/", 1)[1], "actor": actor.get_actor_label(), "mesh": path,
                                      "problem": problem, "triangles": tris, "with_area": good}) + "\n")
                if @FIX@:
                    settings = unreal.EditorStaticMeshLibrary.get_lod_build_settings(mesh, 0)
                    settings.distance_field_resolution_scale = 0.0
                    unreal.EditorStaticMeshLibrary.set_lod_build_settings(mesh, 0, settings)
                    unreal.EditorAssetLibrary.save_loaded_asset(mesh, False)
                    fixed += 1
        unreal.log("meshscan: {} done, {} meshes so far".format(tile, len(seen)))
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("meshscan: finished @RUN@, {} meshes, {} fixed".format(len(seen), fixed))
