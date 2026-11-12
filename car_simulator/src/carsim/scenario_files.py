"""
Reading the files of a scenario folder.

Convention: scenarios/<name>/ holds exactly one ScenarioRunner XML config
(one or more <scenario> variants). Variants of type ComposedScenario are built
from blocks (src/carsim/blocks/) by the shared scenarios/_composed/composed_scenario.py,
so the folder needs nothing else; a folder that has a Python file of its own
(exactly one, not starting with "_") uses that for a custom scenario class.
validate() checks the naming rules ScenarioRunner needs and the block elements.
"""

import ast
import importlib.util
import re
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from functools import lru_cache

from carsim.blocks.spec import check_params
from carsim.paths import SCENARIO_RUNNER_ROOT, SCENARIOS_DIR, scenario_dir

# The shared scenario file and the class in it that builds a scenario from blocks
COMPOSED_SCENARIO_FILE = SCENARIOS_DIR / "_composed" / "composed_scenario.py"
COMPOSED_TYPE = "ComposedScenario"

# <scenario> children that ScenarioRunner handles itself (<other_actor> goes to config.other_actors, which
# ComposedScenario ignores); the rest end up in config.other_parameters
_BUILTIN_TAGS = ("ego_vehicle", "weather", "route", "other_actor")


@dataclass
class Variant:
    name: str
    type: str
    town: str
    ego: dict  # x, y, z, yaw as floats
    params: dict = field(default_factory=dict)  # tag -> attributes of the extra elements, e.g. params["ego_control"]["mode"]
    duplicates: tuple = ()  # tags of the extra elements that appear more than once (params keeps the last one)
    other_actors: int = 0  # number of <other_actor> elements


def _single(folder, pattern):
    files = sorted(folder.glob(pattern))
    if len(files) != 1:
        raise FileNotFoundError(f"Expected exactly one {pattern} in {folder}, found {len(files)}")
    return files[0]


def config_file(name):
    return _single(scenario_dir(name), "*.xml")


def scenario_file(name):
    """
    The Python file ScenarioRunner loads the scenario class from: the folder's own .py if it has one (a custom
    scenario), else the shared scenarios/_composed/composed_scenario.py.
    """
    files = [f for f in scenario_dir(name).glob("*.py") if not f.name.startswith("_")]
    if len(files) > 1:
        raise FileNotFoundError(f"Expected at most one scenario .py in {scenario_dir(name)}, found {len(files)}")
    if files:
        return files[0]
    if not COMPOSED_SCENARIO_FILE.is_file():
        raise FileNotFoundError(f"{COMPOSED_SCENARIO_FILE} not found, and {scenario_dir(name)} has no scenario .py")
    return COMPOSED_SCENARIO_FILE


def variants(name):
    """All <scenario> entries of a scenario folder's XML config."""
    result = []
    for node in ET.parse(config_file(name)).getroot().iter("scenario"):
        ego = node.find("ego_vehicle")
        ego_pose = {k: float(ego.get(k, 0.0)) for k in ("x", "y", "z", "yaw")} if ego is not None else {}
        extra = [child for child in node if child.tag not in _BUILTIN_TAGS]
        params = {child.tag: dict(child.attrib) for child in extra}  # like ScenarioRunner: the last element of a tag wins
        counts = Counter(child.tag for child in extra)
        duplicates = tuple(tag for tag, n in counts.items() if n > 1)
        other_actors = sum(1 for child in node if child.tag == "other_actor")
        result.append(Variant(node.get("name"), node.get("type"), node.get("town"), ego_pose, params, duplicates,
                              other_actors))
    return result


def set_ego_pose(path, variant_name, x, y, z, yaw):
    """
    Rewrite the ego_vehicle x/y/z/yaw of one variant in place, keeping the
    rest of the file (comments, formatting) untouched. Attributes the tag
    does not have yet are added.
    """
    text = path.read_text(encoding="utf-8")
    block = re.search(r'<scenario\b[^>]*\bname="%s".*?</scenario>' % re.escape(variant_name), text, re.S)
    if block is None:
        raise ValueError(f"Scenario '{variant_name}' not found in {path}")

    def fix(match):
        tag = match.group(0)
        for key, value in (("x", x), ("y", y), ("z", z), ("yaw", yaw)):
            attr = '%s="%.2f"' % (key, value)
            if re.search(r'\b%s="[^"]*"' % key, tag):
                tag = re.sub(r'\b%s="[^"]*"' % key, attr, tag)
            else:
                tag = re.sub(r"\s*/>$", lambda m: " %s/>" % attr, tag)
        return tag

    new_block, count = re.subn(r"<ego_vehicle\b[^>]*/>", fix, block.group(0), count=1)
    if count == 0:
        raise ValueError(f"Scenario '{variant_name}' in {path} has no self-closing <ego_vehicle .../> element")
    path.write_text(text[:block.start()] + new_block + text[block.end():], encoding="utf-8")


# --- Validation -------------------------------------------------------------

def _parse(path):
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _defined_classes(path):
    return {n.name for n in _parse(path).body if isinstance(n, ast.ClassDef)}


@lru_cache(maxsize=None)
def _scenario_runner_names(root):
    """
    Names ScenarioRunner's class lookup could return from srunner/scenarios/*.py:
    classes defined there and names those modules import (it matches every
    class in a module's namespace). Read with ast, nothing is imported.
    """
    names = set()
    for path in (root / "srunner" / "scenarios").glob("*.py"):
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.ClassDef):
                names.add(node.name)
            elif isinstance(node, ast.ImportFrom) and node.module != "typing":
                # capitalized, not CONSTANT: a cheap stand-in for "is a class"
                names.update((a.asname or a.name) for a in node.names if a.name[:1].isupper() and not a.name.isupper())
    return frozenset(names)


@lru_cache(maxsize=None)
def _example_variant_names(root):
    names = set()
    for path in (root / "srunner" / "examples").glob("*.xml"):
        try:
            names.update(n.get("name") for n in ET.parse(path).getroot().iter("scenario"))
        except ET.ParseError:
            pass
    return frozenset(names)


def validate(name):
    """
    Check scenarios/<name>/ against ScenarioRunner's naming rules. Returns a
    list of problem descriptions, empty when the scenario is fine.
    """
    try:
        py = scenario_file(name)
        all_variants = variants(name)
        classes = _defined_classes(py)
    except (FileNotFoundError, ET.ParseError, SyntaxError) as e:
        return [str(e)]

    problems = []
    srunner_scenarios = SCENARIO_RUNNER_ROOT / "srunner" / "scenarios"
    if not srunner_scenarios.is_dir():
        problems.append(f"ScenarioRunner not found at {SCENARIO_RUNNER_ROOT}; name collisions were not checked.")
        known_names, example_names = frozenset(), frozenset()
    else:
        known_names = _scenario_runner_names(SCENARIO_RUNNER_ROOT)
        example_names = _example_variant_names(SCENARIO_RUNNER_ROOT)

    # (b) ScenarioRunner imports the file by its basename, so it must not shadow another module
    module = py.stem
    if (srunner_scenarios / f"{module}.py").is_file():
        problems.append(f"Module name '{module}' equals srunner/scenarios/{module}.py; rename {py.name}.")
    else:
        try:
            spec = importlib.util.find_spec(module)
        except (ImportError, ValueError):
            spec = None
        if spec is not None and spec.origin != str(py):
            problems.append(f"Module name '{module}' shadows an importable module ({spec.origin or 'built-in'}); "
                            f"rename {py.name}.")

    seen = set()
    for v in all_variants:
        if v.name in seen:
            problems.append(f"Variant name '{v.name}' is used more than once.")
        seen.add(v.name)
        # (a) the first class ScenarioRunner finds with this name wins, ours comes last
        if v.type in known_names:
            problems.append(f"Variant '{v.name}': type '{v.type}' is also a class name in ScenarioRunner's "
                            "srunner/scenarios modules and would run that instead; rename the class and the type.")
        # (c)
        if v.name in example_names:
            problems.append(f"Variant name '{v.name}' is also used in ScenarioRunner's srunner/examples; rename it.")
        # (d)
        if v.type not in classes:
            problems.append(f"Variant '{v.name}': type '{v.type}' is not a class defined in {py.name} "
                            f"(found: {', '.join(sorted(classes)) or 'none'}).")
        # (e) the block elements of a composed scenario
        if v.type == COMPOSED_TYPE:
            problems.extend(f"Variant '{v.name}': {p}"
                            for p in check_params(v.params, v.duplicates, v.other_actors))
    return problems
