"""Junctions: from the Grasshopper model to the joint editor and back.

A *junction* is a place where three or more beams meet. Pairwise joints (L, T,
X) are handled by the joint rules in Grasshopper; junctions are where you design
a joint of your own.

The round trip:

1. `find_junctions()` - every junction in the model, numbered.
2. `export_junction()` - writes `<name>.fixture.json` (the situation, for the
   joint editor) and `<name>.py` (a joint module to start from, made from
   `joint_module.py.template`) next to the Grasshopper document.
3. You design the joint in VS Code with the joint editor.
4. `joint_rule()` - loads `<name>.py` back into Grasshopper as a rule for that
   junction, so the joint shows up in the whole structure.

Runs in Rhino 8 (Python 3.9) and in plain Python; `watch()` needs Rhino.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import keyword
import os
import re
import subprocess
import sys
import types

from compas.geometry import Point, centroid_points
from compas.tolerance import TOL
from compas_timber.connections import Joint, JointTopology
from compas_timber.elements import Beam
from compas_timber.model import TimberModel
from timber_design.workflow import DirectRule, get_clusters_from_model

try:
    import Grasshopper
    import Rhino
    import scriptcontext
    import System
except ImportError:
    Grasshopper = Rhino = scriptcontext = System = None

FIXTURE_SUFFIX = ".fixture.json"
MODULE_TEMPLATE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "joint_module.py.template"
)
DEFAULT_PARAMS = {
    "depth": 20.0
}  # the parameters a new joint module and its Fixture start with
RESERVED_NAMES = ("compas", "compas_timber", "debug", "joint_editor")


class Junction:
    """Three or more beams meeting at one point.

    Attributes
    ----------
    index : int
        The junction's number, stable as long as the design does not change.
    beams : list[Beam]
        The beams, in the order the joint's constructor takes them.
    beam_indices : list[int]
        Where each beam sits in the list the junctions were found in.
    roles : list[str]
        One constructor parameter name per beam, made from its category.
    location : Point
        Where the beams meet.
    topology : int
        `JointTopology.TOPO_Y` (all beams end here) or `TOPO_K` (at least one passes through).
    """

    def __init__(self, index, beams, beam_indices, location, topology):
        self.index = index
        self.beams = beams
        self.beam_indices = beam_indices
        self.location = location
        self.topology = topology
        self.roles = _roles_from_categories(beams)

    def ends_here(self, beam):
        """Whether `beam` ends at this junction rather than passing through it."""
        _, end = beam.endpoint_closest_to_point(self.location)
        return end.distance_to_point(self.location) <= max(beam.width, beam.height)

    def __repr__(self):
        return f"Junction({self.index}, {len(self.beams)} beams, {JointTopology.get_name(self.topology)})"

    def ToString(self):  # what Grasshopper shows in a panel
        return repr(self)


def find_junctions(beams, min_beams=3, max_distance=None):
    """Find the places where `min_beams` or more beams meet.

    Parameters
    ----------
    beams : list[Beam]
        All beams of the design.
    min_beams : int
        The fewest beams that make a junction.
    max_distance : float, optional
        How far apart centerlines may be and still count as meeting.

    Returns
    -------
    list[Junction]
        Sorted by location (x, then y, then z), numbered from 0.
    """
    # Bare copies: the originals may already carry the Model component's features (slow to copy, and they break on proxies).
    model = TimberModel()
    copies = [
        Beam(beam.frame.copy(), beam.length, beam.width, beam.height) for beam in beams
    ]
    for copy in copies:
        model.add_element(copy)
    index_of = {id(copy): i for i, copy in enumerate(copies)}

    junctions = []
    for cluster in get_clusters_from_model(
        model, max_distance=max_distance or TOL.absolute
    ):
        if len(cluster.elements) < min_beams:
            continue
        indices = sorted(index_of[id(e)] for e in cluster.elements)
        location = Point(
            *centroid_points([candidate.location for candidate in cluster.joints])
        )
        junctions.append(
            Junction(
                None, [beams[i] for i in indices], indices, location, cluster.topology
            )
        )

    junctions.sort(key=lambda junction: _rounded(junction.location))
    for index, junction in enumerate(junctions):
        junction.index = index
    return junctions


def _roles_from_categories(beams):
    """`post`, `edge_a`, `edge_b`: a beam's category, lettered when it repeats."""
    categories = [
        _identifier(beam.attributes.get("category") or "beam") for beam in beams
    ]
    roles, seen = [], {}
    for category in categories:
        if categories.count(category) == 1:
            roles.append(category)
        else:
            seen[category] = seen.get(category, -1) + 1
            roles.append("{}_{}".format(category, "abcdefghijklmnop"[seen[category]]))
    return roles


def _identifier(text):
    text = (
        "".join(c if c.isalnum() else "_" for c in str(text).strip().lower()).strip("_")
        or "beam"
    )
    return "beam_" + text if text[0].isdigit() or keyword.iskeyword(text) else text


def export_junction(folder, name, junction, reach=600.0):
    """Write `<name>.fixture.json`, and `<name>.py` if it isn't there yet.

    The Fixture is always rewritten, since it comes from the design: change the
    design, export again, and the editor runs against the new situation (any
    `params` you set in the old Fixture are kept). Only for the same roles,
    though: other roles mean another junction, whose beams the module can't
    use. The module is never overwritten - from the moment it exists, it is
    your code.

    Returns
    -------
    tuple[str, str, bool]
        The module's path, the Fixture's path, and whether the module was just created.
    """
    _check_name(name)
    module_path = os.path.join(folder, name + ".py")
    fixture_path = os.path.join(folder, name + FIXTURE_SUFFIX)

    params = dict(DEFAULT_PARAMS)
    if os.path.exists(fixture_path):
        with open(fixture_path) as f:
            existing = json.load(f)
        _check_same_roles(name, existing.get("role_bindings", {}), junction.roles)
        params.update(existing.get("params", {}))
    with open(fixture_path, "w") as f:
        f.write(_to_json(junction_fixture(junction, reach, params)) + "\n")

    created = not os.path.exists(module_path)
    if created:
        with open(module_path, "w") as f:
            f.write(joint_module_source(name, junction))
    return module_path, fixture_path, created


def _check_name(name):
    if not name or not name.isidentifier() or keyword.iskeyword(name):
        raise ValueError(
            f"'{name}' can't be a module name. Use lowercase words and underscores, like corner_node."
        )
    if name.endswith("_lib"):
        raise ValueError(
            "Names ending in _lib are reserved for the workshop's libraries."
        )
    if name in sys.builtin_module_names or name in RESERVED_NAMES:
        raise ValueError(
            f"'{name}' is already the name of a module Python knows. Pick another."
        )


def _check_same_roles(name, existing_roles, roles):
    if existing_roles and set(existing_roles) != set(roles):
        raise ValueError(
            "'{0}' is already a junction of {1}, and this one is {2}. Export it under another name, "
            "or delete {0}{3} if you meant to replace it.".format(
                name, ", ".join(existing_roles), ", ".join(roles), FIXTURE_SUFFIX
            )
        )


def _to_json(data):
    """Indented JSON, with each list of numbers (a point) on one line."""
    text = json.dumps(data, indent=2)
    return re.sub(
        r"\[\s*([^\[\]{}]*?)\s*\]",
        lambda m: "[" + ", ".join(v.strip() for v in m.group(1).split(",")) + "]",
        text,
    )


def junction_fixture(junction, reach=600.0, params=None):
    """The junction as a joint editor Fixture: plain data, centred on the junction.

    Each beam is cropped to `reach` either side of the junction, so the editor
    shows the joint and not the whole structure. Cropping keeps each beam's
    direction, so its reference sides are the same ones as in Grasshopper.
    """
    origin = junction.location
    beams = []
    for beam in junction.beams:
        _check_default_orientation(beam)
        start, end = _cropped_centerline(beam, origin, reach)
        beams.append(
            {
                "centerline": [_rounded(start - origin), _rounded(end - origin)],
                "width": beam.width,
                "height": beam.height,
            }
        )
    return {
        "$schema": "./fixture.schema.json",
        "beams": beams,
        "role_bindings": {role: i for i, role in enumerate(junction.roles)},
        "params": dict(params or {}),
    }


def _check_default_orientation(beam):
    # shortcut: the Fixture has no z_vector yet, so beams must use the default orientation; extend the Fixture schema when needed
    default = Beam.from_centerline(beam.centerline, beam.width, beam.height)
    if not TOL.is_allclose(default.frame.yaxis, beam.frame.yaxis, atol=1e-6):
        raise ValueError(
            "A beam in this junction has a custom z_vector. The joint editor can't show beam orientation yet - leave z_vector empty."
        )


def _cropped_centerline(beam, point, reach):
    """The start and end of `beam`'s centerline, at most `reach` either side of `point`."""
    line = beam.centerline
    length = line.length
    t = line.closest_point(point, return_parameter=True)[1] * length
    return line.point_at(max(0.0, t - reach) / length), line.point_at(
        min(length, t + reach) / length
    )


def _rounded(vector):
    return [round(c, 3) + 0.0 for c in vector]  # + 0.0 turns -0.0 into 0.0


def joint_module_source(name, junction):
    """The source of a joint module for `junction`, ready to run in both the editor and Grasshopper."""
    roles = junction.roles
    descriptions = [
        "beam {} (category '{}'), {}".format(
            index,
            beam.attributes.get("category"),
            "ends here" if junction.ends_here(beam) else "passes through",
        )
        for index, beam in zip(junction.beam_indices, junction.beams)
    ]
    with open(MODULE_TEMPLATE) as f:
        template = f.read()
    return template.format(
        name=name,
        class_name=class_name_for(name),
        index=junction.index,
        count=len(roles),
        topology=JointTopology.get_name(junction.topology),
        beam_table="\n".join(
            f"- `{role}`: {text}" for role, text in zip(roles, descriptions)
        ),
        role_list=", ".join(f"`{role}`" for role in roles),
        param_docs="\n".join(
            f"    {role}\n        {text}" for role, text in zip(roles, descriptions)
        ),
        init_args="\n".join(f"        {role}: Beam | None = None," for role in roles),
        role_tuple=", ".join(roles),
        role_properties="".join(
            f"\n    @property\n    def {role}(self) -> Beam:\n        return self.elements[{i}]\n"
            for i, role in enumerate(roles)
        ),
        depth=DEFAULT_PARAMS["depth"],
    )


def class_name_for(name):
    """`corner_node` -> `CornerNodeJoint` (the joint editor's own convention)."""
    words = "".join(part[:1].upper() + part[1:] for part in name.split("_") if part)
    return words if words.endswith("Joint") else words + "Joint"


def open_in_vscode(path):
    """Open `path` in VS Code, in the window that already has this repo open."""
    if sys.platform == "darwin":
        subprocess.Popen(["open", "-a", "Visual Studio Code", path])
    else:
        subprocess.Popen(
            ["code", path], shell=True
        )  # the VS Code installer puts `code` on the PATH


_loaded = {}  # path -> (mtime, joint class)


def load_joint_class(folder, name):
    """The `Joint` subclass defined in `<folder>/<name>.py`, re-read whenever the file changes."""
    path = os.path.join(folder, name + ".py")
    if not os.path.exists(path):
        raise OSError(f"No joint module {path} - export the junction first.")
    mtime = os.path.getmtime(path)
    cached = _loaded.get(path)
    if cached and cached[0] == mtime:
        return cached[1]

    _ensure_debug()
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)

    classes = [
        obj
        for obj in vars(module).values()
        if isinstance(obj, type) and issubclass(obj, Joint) and obj.__module__ == name
    ]
    if len(classes) != 1:
        raise ValueError(
            f"{path} should define exactly one Joint subclass, found {len(classes)}."
        )
    _loaded[path] = (mtime, classes[0])
    return classes[0]


def _ensure_debug():
    """Joint modules `import debug` (the joint editor's drawing channel). Outside the editor it only has to import."""
    if "debug" not in sys.modules and importlib.util.find_spec("debug") is None:
        stub = types.ModuleType("debug")
        stub.show = lambda geometry, label: None
        sys.modules["debug"] = stub


def joint_rule(joint_class, junction, **params):
    """A rule that joins `junction`'s beams with `joint_class`, ahead of every category and topology rule."""
    if len(junction.beams) != joint_class.MAX_ELEMENT_COUNT:
        raise ValueError(
            f"{joint_class.__name__} joins {joint_class.MAX_ELEMENT_COUNT} beams, but junction {junction.index} has {len(junction.beams)}."
        )
    return DirectRule(
        joint_class, _beams_in_constructor_order(joint_class, junction), **params
    )


def _beams_in_constructor_order(joint_class, junction):
    """By role name where the constructor still uses the exported names; by position otherwise."""
    names = inspect.getfullargspec(joint_class.__init__).args[
        1 : 1 + len(junction.beams)
    ]
    beam_by_role = dict(zip(junction.roles, junction.beams))
    return (
        [beam_by_role[n] for n in names]
        if set(names) == set(beam_by_role)
        else list(junction.beams)
    )


def watch(component, folder, names):
    """Re-solve `component` whenever one of the modules `names` is saved in `folder`."""
    key = f"junction_lib.watch.{component.InstanceGuid}"
    entry = scriptcontext.sticky.get(key)
    if entry is None or entry["folder"] != folder:
        if entry is not None:
            entry["watcher"].Dispose()
        entry = {
            "folder": folder,
            "names": set(),
            "watcher": _watch_folder(component, folder, lambda: entry["names"]),
        }
        scriptcontext.sticky[key] = entry
    entry["names"] = {n + ".py" for n in names}


def _watch_folder(component, folder, watched_names):
    """A FileSystemWatcher that schedules `component` to re-solve when a file in `watched_names()` changes."""

    def expire(doc):
        component.ExpireSolution(False)

    def on_ui_thread():
        document = component.OnPingDocument()
        if document is not None:
            document.ScheduleSolution(
                300, Grasshopper.Kernel.GH_Document.GH_ScheduleDelegate(expire)
            )

    def on_changed(sender, args):
        if os.path.basename(args.FullPath) in watched_names():
            Rhino.RhinoApp.InvokeOnUiThread(System.Action(on_ui_thread))

    watcher = System.IO.FileSystemWatcher(folder, "*.py")
    watcher.NotifyFilter = System.IO.NotifyFilters.LastWrite
    watcher.Changed += on_changed
    watcher.EnableRaisingEvents = True
    return watcher
