# Runs inside the editor through cmd.py: generated walls come out grey, so each building gets one of CARLA's facades.
# vm.sh facades replaces the placeholders. It goes tile by tile and sets the facade on each building in the level (a
# material override saved with the tile): on WolfsburgHighway the building meshes load only through their tile, so
# they can't be changed as assets (Wolfsburg2's could). Running it again skips buildings that already have one.
import os

FOLDER = "/Game/Carla/Static/Building/01_ProceduralBuildings/FacadeMaterials"
DISK = "/work/carla/Unreal/CarlaUE4/Content/CustomMaps/@NAME@"

facades = [unreal.load_asset(a) for a in unreal.EditorAssetLibrary.list_assets(FOLDER, recursive=False) if "MI_Facade" in a]
tiles = sorted("/Game/CustomMaps/@NAME@/" + f[:-5] for f in os.listdir(DISK) if f.endswith(".umap") and "_Tile_" in f)
total = 0
for tile in tiles:
    unreal.EditorLevelLibrary.load_level(tile)
    n = 0
    for actor in unreal.EditorLevelLibrary.get_all_level_actors():
        for comp in actor.get_components_by_class(unreal.StaticMeshComponent):
            mesh = comp.static_mesh
            if mesh is None or "/Static/Buildings/" not in mesh.get_path_name():
                continue
            current = comp.get_material(0)
            if current is not None and "MI_Facade" in current.get_name():
                continue
            comp.set_material(0, facades[sum(map(ord, mesh.get_name())) % len(facades)])  # same facade every run
            n += 1
    if n:
        unreal.EditorLevelLibrary.save_current_level()
    unreal.log("facades: {} {} buildings".format(tile.rsplit("/", 1)[1], n))
    total += n
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("facades: finished @RUN@, {} set".format(total))
