#!/usr/bin/env python3
"""Applies the small fixes the Wolfsburg build needs to CARLA 0.9.16 and its Unreal. GUIDE.md explains each one.

Run it after fetching Unreal and again after cloning CARLA. Running it twice is safe.
Every line it prints starts with "ok" or "not yet" ("not yet": that folder doesn't exist yet).
It checks every patch before writing anything: if any file isn't as expected, it says which and changes nothing.
Usage: apply_patches.py [UE4 folder] [CARLA folder]    or    apply_patches.py --selftest
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

OSM = "std::string OsmFile = std::string(TCHAR_TO_UTF8(*FileContent));"
UE4_DIR, CARLA_DIR = "ue4", "carla"
TOOL = "Unreal/CarlaUE4/Plugins/CarlaTools/Source/CarlaTools/Private/OpenDriveToMap.cpp"
MISC = "TArray<AActor*> UOpenDriveToMap::GenerateMiscActors(float Offset, FVector MinLocation, FVector MaxLocation )\n{"
# The stock tool puts every tree, street light, fire hydrant and light box 3 m past a road's last driving lane, for the
# whole road in every tile the road touches. So spots landed on sidewalks, slip roads, crossing roads and buildings,
# repeated in each tile, and a tree beside a ramp had its crown, which collides, at deck height (the A 39 launch).
# PlaceSpot keeps a spot only inside its own tile, moves it ClearOfRoad from the nearest driving lane, and drops it
# if it is then near any other lane or a building outline in <map>.keepout (written by prepare_dt.py).
SPOT_FREE = r"""// Patched (apply_patches.py); prepare_dt.py replays this before Generate and check_level.py checks the result.
static bool PlaceSpot(const carla::road::Map& Map, carla::geom::Location& Spot, const FString& Name,
    const FVector& TileA, const FVector& TileB)
{
  constexpr double ClearOfRoad = 4.5;   // m to a driving lane's edge: tree crowns collide and are up to 8.2 m wide
  constexpr double ClearOfLanes = 1.0;  // m to any other lane's edge (sidewalks)
  constexpr double ClearOfWalls = 3.0;  // m to a building wall
  const auto In = [](double V, double A, double B) { return V >= FMath::Min(A, B) && V < FMath::Max(A, B); };
  if (!In(Spot.x * 100.0, TileA.X, TileB.X) || !In(Spot.y * 100.0, TileA.Y, TileB.Y))
  {
    return false;  // another tile places it
  }
  const auto Gap = [&Map, &Spot](carla::road::Lane::LaneType Type, carla::geom::Location* Centre) {
    const auto W = Map.GetClosestWaypointOnRoad(Spot, static_cast<int32_t>(Type));
    if (!W) return 1e9;
    const auto C = Map.ComputeTransform(*W).location;
    if (Centre) *Centre = C;
    return carla::geom::Math::Distance2D(C, Spot) - Map.GetLaneWidth(*W) * 0.5;
  };
  for (int Step = 0; Step < 3; ++Step)  // a move away from one road can bring it near another
  {
    carla::geom::Location C;
    const double G = Gap(carla::road::Lane::LaneType::Driving, &C);
    if (G >= ClearOfRoad) break;
    FVector2D Away(Spot.x - C.x, Spot.y - C.y);
    const double Dist = Away.Size();
    if (Dist < 0.01) return false;
    Away /= Dist;
    Spot.x = C.x + Away.X * (Dist - G + ClearOfRoad + 0.05);
    Spot.y = C.y + Away.Y * (Dist - G + ClearOfRoad + 0.05);
  }
  if (Gap(carla::road::Lane::LaneType::Driving, nullptr) < ClearOfRoad || Gap(carla::road::Lane::LaneType::Any, nullptr) < ClearOfLanes)
  {
    return false;
  }
  static FString Loaded;
  static TArray<TArray<FVector2D>> Outlines;  // one building per line of x y x y ... in CARLA metres
  if (Loaded != Name)
  {
    Loaded = Name;
    Outlines.Reset();
    TArray<FString> Lines;
    FFileHelper::LoadFileToStringArray(Lines, *(FPaths::ProjectContentDir() + "CustomMaps/" + Name + "/OpenDrive/" + Name + ".keepout"));
    for (const FString& Line : Lines)
    {
      TArray<FString> N;
      Line.ParseIntoArrayWS(N);
      TArray<FVector2D>& O = Outlines.AddDefaulted_GetRef();
      for (int i = 0; i + 1 < N.Num(); i += 2) O.Add(FVector2D(FCString::Atof(*N[i]), FCString::Atof(*N[i + 1])));
    }
    UE_LOG(LogCarlaToolsMapGenerator, Warning, TEXT("PlaceSpot: %d building outlines for %s"), Outlines.Num(), *Name);
  }
  const FVector P(Spot.x, Spot.y, 0.0f);
  for (const TArray<FVector2D>& O : Outlines)
  {
    bool Inside = false;
    for (int i = 0, j = O.Num() - 1; i < O.Num(); j = i++)
    {
      if ((O[i].Y > P.Y) != (O[j].Y > P.Y) && P.X < O[i].X + (P.Y - O[i].Y) * (O[j].X - O[i].X) / (O[j].Y - O[i].Y)) Inside = !Inside;
      if (FMath::PointDistToSegment(P, FVector(O[i], 0.0f), FVector(O[j], 0.0f)) < ClearOfWalls) return false;
    }
    if (Inside) return false;
  }
  return true;
}

"""
HEIGHT = "cl.first.location.z{}= GetHeight(cl.first.location.x, cl.first.location.y) + 0.3f;"
# Without a heightmap the stock tool adds made-up waves (Deformation.h: up to 1.2 m, slopes up to 4.5 degrees) to every
# road, lane mark, sidewalk, tree and the ground, so cars pitched and rolled on roads the road file has flat (r = 0.999,
# jiggle_test.py, 2026-11-11). Now everything keeps the road file's heights. (The driving lane "bumps" never applied:
# DistanceToLaneBorder returns metres minus the lane width and is compared with 65.)
WAVES = """    if( bDrivingLane ){
      return carla::geom::deformation::GetZPosInDeformation(PosX, PosY) +
        (carla::geom::deformation::GetZPosInDeformation(PosX, PosY) * -0.3f) -
        carla::geom::deformation::GetBumpDeformation(PosX,PosY);
    }else{
      return carla::geom::deformation::GetZPosInDeformation(PosX, PosY) + (carla::geom::deformation::GetZPosInDeformation(PosX, PosY) * -0.3f);
    }"""
FLAT = "    return 0.0f;  // patched (apply_patches.py): no made-up waves, roads keep the road file's heights"
# The ground is sampled every 2.56 m and dropped 80 cm wherever a sample lies under any mesh: beside every road without
# a sidewalk that left a trench up to 80 cm deep and 2.56 m wide, and a wheel just past the lane edge fell into it.
TRENCH = "return GetHeight(Origin.X * 0.01f, Origin.Y * 0.01f, true) * 100.0f - {}.0f;"
PATCHES = [
    # Engine: written against CarlaUnreal 8d08ab7fe (patches/unreal-4.26-8d08ab7fe.diff). CARLA 0.9.16 builds with the
    # head of CarlaUnreal's carla branch, not checked here; if its text differs, the script stops before any write.
    (UE4_DIR, "Engine/Source/Runtime/Engine/Private/DistanceFieldAtlas.cpp",
     "OldVolumeData->VolumeTexture.Release();",
     "if (OldVolumeData) OldVolumeData->VolumeTexture.Release();"),
    (UE4_DIR, "Engine/Source/Runtime/Engine/Private/DistanceFieldAtlas.cpp",
     "BeginCleanup(OldVolumeData);",
     "if (OldVolumeData) BeginCleanup(OldVolumeData);"),
    (UE4_DIR, "Engine/Source/Developer/MeshMergeUtilities/Private/MeshMergeUtilities.cpp",
     "const bool bMergeMaterialData = InSettings.bMergeMaterials && "
     "InSettings.LODSelectionType != EMeshLODSelectionType::AllLODs;",
     "const bool bMergeMaterialData = false;"),
    (UE4_DIR, "Engine/Source/Developer/MeshMergeUtilities/Private/MeshMergeUtilities.cpp",
     '#include "MeshMergeUtilities.h"',
     '#include "MeshMergeUtilities.h"\n#include "Materials/MaterialInstance.h"'),
    (UE4_DIR, "Engine/Source/Developer/MeshMergeUtilities/Private/MeshMergeUtilities.cpp",
     "Material = nullptr; // do not save non-asset materials",
     "UMaterialInstance* Instance = Cast<UMaterialInstance>(Material); "
     "Material = Instance ? Instance->Parent : nullptr; // walls use dynamic materials: keep the parent"),
    # Fixed upstream in 0.9.16 (boost URL, roads without a speed record, meshes without a material). Kept as checks
    # (old = new), so an older CARLA stops here instead of being built without them.
    (CARLA_DIR, "Util/BuildTools/Setup.sh", "https://archives.boost.io/release/", "https://archives.boost.io/release/"),
    (CARLA_DIR, "LibCarla/source/carla/road/Map.cpp",
     'transforms.push_back(std::make_pair(treeTransform, "urban"));',
     'transforms.push_back(std::make_pair(treeTransform, "urban"));'),
    (CARLA_DIR, "Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Commandlet/PrepareAssetsForCookingCommandlet.cpp",
     "Material = UMaterial::GetDefaultMaterial(MD_Surface);",
     "Material = UMaterial::GetDefaultMaterial(MD_Surface);"),
    (CARLA_DIR, "Util/BuildTools/Setup.sh",
     "libpng16/${LIBPNG_VERSION}/",
     "libpng16/older-releases/${LIBPNG_VERSION}/"),
    # The tool converts the downloaded .osm and overwrites the .xodr; a prepared one (vm.sh prepared) is kept instead.
    (CARLA_DIR, "Unreal/CarlaUE4/Plugins/CarlaTools/Source/CarlaTools/Private/Online/CustomFileDownloader.cpp",
     OSM,
     'FString Prepared = FilePath.Replace(TEXT(".osm"), TEXT(".xodr")); '
     "if (FileManager.FileExists(*Prepared)) { return; }\n  " + OSM),
    # Junctions of 3+ roads get a signed-distance surface that only exists near z = 0, so one on a ramp or bridge got
    # none and cars fell through it. Raised ones use the per-lane meshes instead, which follow the road's height.
    (CARLA_DIR, "LibCarla/source/carla/road/Map.cpp",
     "      if (junction.GetConnections().size() > 2) {",
     "      double JunctionTop = 0.0;  // patched (apply_patches.py): highest point of the junction's roads\n"
     "      for (const auto& c : junction.GetConnections()) {\n"
     "        const auto& r = _data.GetRoads().at(c.second.connecting_road);\n"
     "        for (double f : {0.0, 0.5, 1.0}) {\n"
     "          JunctionTop = std::max(JunctionTop, std::abs(static_cast<double>(r.GetDirectedPointIn(f * r.GetLength()).location.z)));\n"
     "        }\n"
     "      }\n"
     "      if (junction.GetConnections().size() > 2 && JunctionTop < 0.3) {"),
    (CARLA_DIR, TOOL, MISC, SPOT_FREE + MISC),
    (CARLA_DIR, TOOL, HEIGHT.format(" "),  # street lights, fire hydrants, light boxes
     "if (!PlaceSpot(*CarlaMap, cl.first.location, MapName, MinLocation, MaxLocation)) continue;\n    " + HEIGHT.format(" ")),
    (CARLA_DIR, TOOL, HEIGHT.format("  "),  # trees
     "if (!PlaceSpot(*ParamCarlaMap, cl.first.location, MapName, MinLocation, MaxLocation)) continue;\n    " + HEIGHT.format("  ")),
    (CARLA_DIR, TOOL, WAVES, FLAT),
    (CARLA_DIR, TOOL, TRENCH.format(80),
     TRENCH.format(12) + "  // patched (apply_patches.py): 80 left a trench beside every road"),
]


def patch(data, old, new):
    """Returns the patched bytes and what happened. Works on bytes so nothing else in the file changes."""
    old, new = old.encode(), new.encode()
    if new in data:
        return data, "ok       (already patched)"
    if data.count(old) != 1:
        raise ValueError(f"has {data.count(old)} matches for '{old.decode()}', expected 1")
    return data.replace(old, new), "ok      "


def main(ue4, carla):
    roots = {UE4_DIR: Path(ue4), CARLA_DIR: Path(carla)}
    files, report, problems = {}, [], []
    for root, name, old, new in PATCHES:
        path = roots[root] / name
        if not roots[root].is_dir():  # vm.sh runs this once before CARLA is cloned
            report.append("not yet  " + path.name)
            continue
        try:
            files[path], what = patch(files[path] if path in files else path.read_bytes(), old, new)
            report.append(what + " " + path.name)
        except (OSError, ValueError) as e:
            problems.append(f"{path}: {e}")
    if problems:
        sys.exit("\n".join(problems) + "\nNothing was changed (patches/*.diff show what each patch does). "
                 "Stop and tell Claude.")
    for path, data in files.items():
        if data != path.read_bytes():
            path.write_bytes(data)
    print("\n".join(report))


def selftest():
    assert patch(b"a old b", "old", "if (y) old") == (b"a if (y) old b", "ok      ")
    assert "already" in patch(b"a if (y) old b", "old", "if (y) old")[1]
    try:
        patch(b"moved", "old", "new")
    except ValueError as e:
        assert "expected 1" in str(e)
    else:
        raise AssertionError("a changed file must stop the script")
    # a CARLA tree holding every anchor once, and no Unreal yet
    with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
        carla = Path(d) / "carla"
        for root, name, old, _ in PATCHES:
            if root == CARLA_DIR:
                (carla / name).parent.mkdir(parents=True, exist_ok=True)
                with open(carla / name, "a") as f:
                    f.write(old + "\n")
        tree = lambda: {p: p.read_bytes() for p in carla.rglob("*") if p.is_file()}
        broken = carla / PATCHES[-1][1]
        before = broken.read_bytes()
        broken.write_bytes(before.replace(PATCHES[-1][2].encode(), b"moved"))
        stock = tree()
        try:
            main(Path(d) / "UE4", carla)
        except SystemExit as e:
            assert "expected 1" in str(e)
            assert tree() == stock, "one changed anchor must leave every file as it was"
        else:
            raise AssertionError("a changed file must stop the script")
        broken.write_bytes(before)
        main(Path(d) / "UE4", carla)
        once = tree()
        assert once != stock
        main(Path(d) / "UE4", carla)
        assert tree() == once, "a second run must change nothing"
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        selftest()
    else:
        main(*(sys.argv[1:] or ["/work/UE4", "/work/carla"]))
