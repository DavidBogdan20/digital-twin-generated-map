# Runs inside the editor through cmd.py: fills in the Digital Twin Tool and presses Generate.
# vm.sh generate replaces the two placeholders. The URL must carry bbox=, because the tool reads the map origin from it.
fields = {}
for field in unreal.ObjectIterator(unreal.EditableText):
    path = field.get_path_name()
    if "UW_OnRoad" in path and "EmptyMap" in path:
        fields[path.split(".")[-1]] = field
fields["OSMURL"].set_text("@URL@")
fields["Filename"].set_text("@NAME@")
tool.call_method("BndEvt__UWG_OSMDownloader_DownloadButton_K2Node_ComponentBoundEvent_0_OnButtonClickedEvent__DelegateSignature")
unreal.log("generate: pressed for @NAME@")
