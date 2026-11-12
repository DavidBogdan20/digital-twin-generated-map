"""
Composable scenario blocks: small classes keyed by an XML tag that scenarios/_composed/composed_scenario.py
(ComposedScenario) builds from a variant's elements. See carsim.blocks.base for the interface.

Importing this package is cheap (the tag lists in carsim.blocks.spec need no CARLA); the block classes are imported
by block_classes() / create_blocks(), which need CARLA and ScenarioRunner on the import path.
"""

import importlib

from carsim.blocks.spec import BLOCK_ORDER, BLOCK_PARAMS, CROSSING_BLOCKS, IMPLICIT_BLOCKS, check_params

__all__ = ["BLOCK_ORDER", "BLOCK_PARAMS", "CROSSING_BLOCKS", "IMPLICIT_BLOCKS", "block_classes", "check_params",
           "create_blocks"]

# tag -> (module, class name)
_BLOCKS = {
    "ego_control": ("carsim.blocks.ego_control", "EgoControl"),
    "spawning": ("carsim.blocks.spawning", "Spawning"),
    "run_control": ("carsim.blocks.run_control", "RunControl"),
    "occluder": ("carsim.blocks.occluder", "Occluder"),
    "pedestrian": ("carsim.blocks.pedestrian", "Pedestrian"),
    "crowd": ("carsim.blocks.crowd", "Crowd"),
    "cyclists": ("carsim.blocks.cyclists", "Cyclists"),
    "cut_in": ("carsim.blocks.cut_in", "CutIn"),
    "background": ("carsim.blocks.background", "Background"),
    "end": ("carsim.blocks.end", "End"),
    "metrics": ("carsim.blocks.metrics", "Metrics"),
}


def block_classes():
    """{tag: Block subclass} for every block, in BLOCK_ORDER."""
    classes = {}
    for tag in BLOCK_ORDER:
        module, name = _BLOCKS[tag]
        classes[tag] = getattr(importlib.import_module(module), name)
    return classes


def create_blocks(ctx, params):
    """
    The blocks of a scenario, as a list in BLOCK_ORDER. `params` is config.other_parameters: tag -> attributes.
    A block exists when its tag is in `params`; ego_control and end always exist (with default parameters).
    Sets ctx.uses_crossing. Raises ValueError for an unknown tag.
    """
    unknown = [tag for tag in params if tag not in BLOCK_PARAMS]
    if unknown:
        raise ValueError("Unknown block element(s) <{}>; known blocks: {}".format(
            ">, <".join(unknown), ", ".join(BLOCK_ORDER)))
    classes = block_classes()
    blocks = [classes[tag](ctx, params.get(tag, {})) for tag in BLOCK_ORDER
              if tag in params or tag in IMPLICIT_BLOCKS]
    ctx.uses_crossing = any(block.needs_crossing for block in blocks)
    return blocks
