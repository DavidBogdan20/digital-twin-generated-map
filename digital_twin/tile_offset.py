# Runs inside the editor through cmd.py (vm.sh tileoffset): makes the map's LargeMapManager find the right tile for a
# location. The Digital Twin Tool lays tile X_Y out from (X, -Y) * TileSide (its south-west corner) and leaves
# Tile0Offset at 0, but ALargeMapManager::GetTileVectorID takes Tile0Offset as tile 0's centre: every lookup was half a
# tile off, so the last 250 m in the north and east counted as a tile that does not exist. Any actor there went dormant
# at once, even with every tile loaded, and a walker going dormant under its AI controller crashed the 0.9.15 server in
# FWalkerActor::SetWalkerState (WolfsburgHighway, cyclist cut-in, 2026-11-10). LocalTileOffset is used only by that
# lookup (not to place tiles), so half a tile there fixes it. Saves the main map; repackage afterwards.
lm = None
unreal.EditorLevelLibrary.load_level("/Game/CustomMaps/@NAME@/@NAME@")
for actor in unreal.EditorLevelLibrary.get_all_level_actors():
    if actor.get_class().get_name() == "LargeMapManager":
        lm = actor
if lm is None:
    unreal.log_error("tileoffset: no LargeMapManager in /Game/CustomMaps/@NAME@/@NAME@")
else:
    side = lm.get_editor_property("tile_side")
    lm.set_editor_property("local_tile_offset", unreal.Vector(side / 2, -side / 2, 0.0) - lm.get_editor_property("tile0_offset"))
    unreal.EditorLevelLibrary.save_current_level()
    unreal.log("tileoffset: TileSide {} Tile0Offset {} LocalTileOffset {}".format(
        side, lm.get_editor_property("tile0_offset"), lm.get_editor_property("local_tile_offset")))
unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
unreal.log("tileoffset: finished @RUN@")
