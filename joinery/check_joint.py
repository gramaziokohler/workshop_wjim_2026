"""Run a joint module against its Fixture, without VS Code or Rhino, and say what the robot can mill.

    uv run python joinery/check_joint.py joinery/corner_node.py

The same Run as the joint editor (Fixture -> beams -> joint -> process_joinery), printed instead of drawn.
Exits 1 if the joint fails anywhere, so an agent can loop on it.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import sys
import types
from pathlib import Path

from compas.geometry import Line
from compas_timber.connections import Joint
from compas_timber.elements import Beam
from compas_timber.model import TimberModel

from toolpath_lib import toolpaths


def load_joint_class(path, preferred=None):
    if importlib.util.find_spec("debug") is None:  # debug.show() draws in the editor only
        sys.modules["debug"] = types.SimpleNamespace(show=lambda *args, **kwargs: None)
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    found = [c for c in vars(module).values() if inspect.isclass(c) and issubclass(c, Joint) and c.__module__ == module.__name__]
    if preferred:
        found = [c for c in found if c.__name__ == preferred]
    if len(found) != 1:
        raise SystemExit(f"expected one Joint subclass in {path.name}, found {[c.__name__ for c in found]}")
    return found[0]


def check(path):
    """Print every failure and every beam's processings with their toolpath. Returns the number of failures."""
    fixture = json.loads(path.with_name(path.stem + ".fixture.json").read_text())
    joint_class = load_joint_class(path, fixture.get("joint_class"))

    model = TimberModel()
    beams = [Beam.from_centerline(Line(*b["centerline"]), width=b["width"], height=b["height"]) for b in fixture["beams"]]
    for beam in beams:
        model.add_element(beam)
    roles = {role: beams[index] for role, index in fixture["role_bindings"].items()}
    model.add_joint(joint_class(**roles, **fixture.get("params", {})))

    failures = [f"joint: {e.debug_info or e}" for e in model.process_joinery()]
    for role, beam in roles.items():
        beam.compute_elementgeometry()  # applies the features; one that fails lands on beam.debug_info
        failures.extend(f"{role}: feature failed: {e.message}" for e in beam.debug_info)
        print(f"{role}: {len(beam.features)} processing(s)")
        for toolpath in toolpaths(beam):
            print(f"  side {toolpath.ref_side_index}: {type(toolpath.processing).__name__}: {toolpath.note}")
    for failure in failures:
        print("FAIL", failure)
    print("OK" if not failures else f"{len(failures)} failure(s)")
    return len(failures)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    sys.exit(1 if check(Path(sys.argv[1]).resolve()) else 0)
