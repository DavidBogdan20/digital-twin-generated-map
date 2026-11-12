# Runs inside the Unreal editor at startup: leaves the demo city for an empty map and opens the Digital Twin Tool.
# It then runs any Python placed in /work/dt/cmd.py, which reaches widget fields the remote screen can't.
import os
import traceback
import unreal

COMMAND_FILE = "/work/dt/cmd.py"
OPEN_AT_TICK = 200  # the editor isn't ready for new tabs on its first frames
ticks = 0
tool = None


def tick(_):
    global ticks, tool
    ticks += 1
    if ticks == OPEN_AT_TICK:
        unreal.EditorLevelLibrary.load_level("/Game/Carla/Maps/TestMaps/EmptyMap")
        widget = unreal.load_asset("/CarlaTools/OnroadMapGenerator/UW_OnRoadMainWidget")
        tool = unreal.get_editor_subsystem(unreal.EditorUtilitySubsystem).spawn_and_register_tab(widget)
        unreal.log("open_tool: tool tab opened: {}".format(tool))
    if ticks % 30 == 0 and os.path.exists(COMMAND_FILE):
        with open(COMMAND_FILE) as f:
            source = f.read()
        os.remove(COMMAND_FILE)
        try:
            exec(source, globals())
        except Exception:
            unreal.log_error(traceback.format_exc())


unreal.register_slate_post_tick_callback(tick)
