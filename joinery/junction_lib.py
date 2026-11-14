"""Junctions: from the Grasshopper model to the joint editor and back.

A *junction* is a place where three or more beams meet. Pairwise joints (L, T,
X) are handled by the joint rules in Grasshopper; junctions are where you design
a joint of your own.

The round trip:

1. `find_junctions()` - every junction in the model, numbered.
2. `export_junction()` - writes `<name>.fixture.json` (the situation, for the
   joint editor) and `<name>.py` (a joint module to start from) next to the
   Grasshopper document.
3. You design the joint in VS Code with the joint editor.
4. `joint_rule()` - loads `<name>.py` back into Grasshopper as a rule for that
   junction, so the joint shows up in the whole structure.

Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import keyword
import os
import re
import sys
import types

from compas.geometry import Point
from compas.geometry import centroid_points
from compas.tolerance import TOL
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.elements import Beam
from compas_timber.model import TimberModel

FIXTURE_SUFFIX = ".fixture.json"


# ---------------------------------------------------------------------------
# 1. Finding junctions
# ---------------------------------------------------------------------------


class Junction(object):
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
        return "Junction({}, {} beams, {})".format(self.index, len(self.beams), JointTopology.get_name(self.topology))

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
    # timber_design is only needed here, and only exists where Grasshopper does
    from timber_design.workflow import get_clusters_from_model

    # Work on bare copies: the originals belong to the Model component's model, and
    # may carry its features already (copying those is slow, and breaks on proxies).
    model = TimberModel()
    copies = [Beam(beam.frame.copy(), beam.length, beam.width, beam.height) for beam in beams]
    for copy in copies:
        model.add_element(copy)
    index_of = {id(copy): i for i, copy in enumerate(copies)}

    found = []
    for cluster in get_clusters_from_model(model, max_distance=max_distance or TOL.absolute):
        if len(cluster.elements) < min_beams:
            continue
        indices = sorted(index_of[id(e)] for e in cluster.elements)
        location = Point(*centroid_points([candidate.location for candidate in cluster.joints]))
        found.append((indices, location, cluster.topology))

    found.sort(key=lambda f: (round(f[1].x, 3), round(f[1].y, 3), round(f[1].z, 3)))
    return [Junction(i, [beams[k] for k in indices], indices, location, topology) for i, (indices, location, topology) in enumerate(found)]


def _roles_from_categories(beams):
    """`post`, `edge_a`, `edge_b`: a beam's category, lettered when it repeats."""
    categories = [_identifier(beam.attributes.get("category") or "beam") for beam in beams]
    roles, seen = [], {}
    for category in categories:
        if categories.count(category) == 1:
            roles.append(category)
        else:
            seen[category] = seen.get(category, -1) + 1
            roles.append("{}_{}".format(category, "abcdefghijklmnop"[seen[category]]))
    return roles


def _identifier(text):
    text = "".join(c if c.isalnum() else "_" for c in str(text).strip().lower()).strip("_") or "beam"
    return "beam_" + text if text[0].isdigit() or keyword.iskeyword(text) else text


# ---------------------------------------------------------------------------
# 2. Exporting a junction to the joint editor
# ---------------------------------------------------------------------------


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
        line = beam.centerline
        length = line.length
        t = line.closest_point(origin, return_parameter=True)[1] * length
        start = line.point_at(max(0.0, t - reach) / length)
        end = line.point_at(min(length, t + reach) / length)
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


def _rounded(vector):
    return [round(c, 3) + 0.0 for c in vector]  # + 0.0 turns -0.0 into 0.0


def _check_default_orientation(beam):
    # shortcut: the Fixture has no z_vector yet, so beams must use the default orientation; extend the Fixture schema when needed
    default = Beam.from_centerline(beam.centerline, beam.width, beam.height)
    if not TOL.is_allclose(default.frame.yaxis, beam.frame.yaxis, atol=1e-6):
        raise ValueError("A beam in this junction has a custom z_vector. The joint editor can't show beam orientation yet - leave z_vector empty.")


def export_junction(folder, name, junction, reach=600.0):
    """Write `<name>.fixture.json`, and `<name>.py` if it isn't there yet.

    The Fixture is always rewritten, since it comes from the design: change the
    design, export again, and the editor runs against the new situation (any
    `params` you set in the old Fixture are kept). The module is never
    overwritten - from the moment it exists, it is your code.

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
            params.update(json.load(f).get("params", {}))
    text = json.dumps(junction_fixture(junction, reach, params), indent=2)
    text = re.sub(r"\[\s*([^\[\]{}]*?)\s*\]", lambda m: "[" + ", ".join(v.strip() for v in m.group(1).split(",")) + "]", text)  # a point per line
    with open(fixture_path, "w") as f:
        f.write(text + "\n")

    created = not os.path.exists(module_path)
    if created:
        with open(module_path, "w") as f:
            f.write(joint_module_source(name, junction))
    return module_path, fixture_path, created


def open_in_vscode(path):
    """Open `path` in VS Code, in the window that already has this repo open."""
    import subprocess

    if sys.platform == "darwin":
        subprocess.Popen(["open", "-a", "Visual Studio Code", path])
    else:
        subprocess.Popen(["code", path], shell=True)  # the VS Code installer puts `code` on the PATH


def _check_name(name):
    if not name or not name.isidentifier() or keyword.iskeyword(name):
        raise ValueError("'{}' can't be a module name. Use lowercase words and underscores, like corner_node.".format(name))
    if name.endswith("_lib"):
        raise ValueError("Names ending in _lib are reserved for the workshop's libraries.")
    if name in sys.builtin_module_names or name in ("compas", "compas_timber", "debug", "joint_editor"):
        raise ValueError("'{}' is already the name of a module Python knows. Pick another.".format(name))


def class_name_for(name):
    """`corner_node` -> `CornerNodeJoint` (the joint editor's own convention)."""
    words = "".join(part[:1].upper() + part[1:] for part in name.split("_") if part)
    return words if words.endswith("Joint") else words + "Joint"


# ---------------------------------------------------------------------------
# 3. Generating a joint module to start from
# ---------------------------------------------------------------------------

# The parameters the generated module starts with, and the Fixture with it.
DEFAULT_PARAMS = {"depth": 20.0}

_MODULE_TEMPLATE = '''"""{class_name}: the joint for junction {index}, where {count} beams meet ({topology}).

Exported from Grasshopper by junction_lib. From here on this file is yours.
`{name}.fixture.json`, next to it, is the situation it was exported from:
open this file in VS Code and the joint editor runs it against those beams.

The beams, in the order the constructor takes them:

{beam_table}

Before writing any code, write down here what this joint should do, in
carpentry words: which beam is cut, which one carries, what each cut is for.
"""

from __future__ import annotations

from itertools import combinations

from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.elements import Beam

# Your building blocks are BTLx processings, for example:
# from compas_timber.fabrication import Lap, Pocket, Drilling, JackRafterCut

import debug


class {class_name}(Joint):
    """Joins {role_list}.

    Parameters
    ----------
{param_docs}
    depth
        An example parameter - rename it, use it, or remove it.
    """

    SUPPORTED_TOPOLOGY = JointTopology.{topology}
    MIN_ELEMENT_COUNT = {count}
    MAX_ELEMENT_COUNT = {count}

    @property
    def __data__(self) -> dict:
        data = super().__data__
        data["depth"] = self.depth
        return data

    # Annotated parameters can be set from the Fixture's "params".
    def __init__(
        self,
{init_args}
        depth: float = {depth},
        **kwargs,
    ) -> None:
        super().__init__(elements=({role_tuple}), **kwargs)
        self.depth = depth
{role_properties}
    @property
    def location(self) -> Point:
        """Where the beams meet: the centre of the closest points between their centerlines."""
        if self._location is None:
            points = []
            for beam_a, beam_b in combinations(self.elements, 2):
                point_a, point_b = intersection_line_line(beam_a.centerline, beam_b.centerline)
                if point_a is not None:
                    points.extend([point_a, point_b])
            self._location = Point(*centroid_points(points))
        return self._location

    def side_facing(self, beam: Beam, other: Beam) -> int:
        """The index of `beam`'s reference side that faces `other`."""
        incidence = beam_ref_side_incidence(other, beam, ignore_ends=True)
        return min(incidence, key=incidence.get)

    def add_extensions(self) -> None:
        """Lengthen beam blanks that must reach past their centerline end. Runs before `add_features()`."""
        pass

    def add_features(self) -> None:
        """Add this joint's processings to its beams. Runs during `process_joinery()`."""
        # Each beam points from the junction back along its centerline.
        for beam in self.elements:
            direction = self.get_beam_direction_towards_joint(beam)
            debug.show(Line(self.location, self.location - direction * 300), "towards joint")
'''


def joint_module_source(name, junction):
    """The source of a joint module for `junction`, ready to run in both the editor and Grasshopper."""
    roles = junction.roles
    beam_table = "\n".join(
        "- `{}`: beam {} (category '{}'), {}".format(
            role,
            index,
            beam.attributes.get("category"),
            "ends here" if junction.ends_here(beam) else "passes through",
        )
        for role, index, beam in zip(roles, junction.beam_indices, junction.beams)
    )
    role_properties = "".join(
        "\n    @property\n    def {}(self) -> Beam:\n        return self.elements[{}]\n".format(role, i) for i, role in enumerate(roles)
    )
    return _MODULE_TEMPLATE.format(
        name=name,
        class_name=class_name_for(name),
        index=junction.index,
        count=len(roles),
        topology=JointTopology.get_name(junction.topology),
        beam_table=beam_table,
        role_list=", ".join("`{}`".format(r) for r in roles),
        param_docs="\n".join("    {}\n        {}".format(r, line.split(": ", 1)[1]) for r, line in zip(roles, beam_table.splitlines())),
        init_args="\n".join("        {}: Beam | None = None,".format(r) for r in roles),
        role_tuple=", ".join(roles),
        role_properties=role_properties,
        depth=DEFAULT_PARAMS["depth"],
    )


# ---------------------------------------------------------------------------
# 4. Loading a joint module back into Grasshopper
# ---------------------------------------------------------------------------

_loaded = {}  # path -> (mtime, joint class)


def _ensure_debug():
    """Joint modules `import debug` (the joint editor's drawing channel). Outside the editor it only has to import."""
    try:
        import debug  # noqa: F401
    except ImportError:
        stub = types.ModuleType("debug")
        stub.show = lambda geometry, label: None
        sys.modules["debug"] = stub


def load_joint_class(folder, name):
    """The `Joint` subclass defined in `<folder>/<name>.py`, re-read whenever the file changes."""
    path = os.path.join(folder, name + ".py")
    if not os.path.exists(path):
        raise IOError("No joint module {} - export the junction first.".format(path))
    mtime = os.path.getmtime(path)
    cached = _loaded.get(path)
    if cached and cached[0] == mtime:
        return cached[1]

    _ensure_debug()
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)

    classes = [obj for obj in vars(module).values() if isinstance(obj, type) and issubclass(obj, Joint) and obj.__module__ == name]
    if len(classes) != 1:
        raise ValueError("{} should define exactly one Joint subclass, found {}.".format(path, len(classes)))
    _loaded[path] = (mtime, classes[0])
    return classes[0]


def joint_rule(joint_class, junction, **params):
    """A rule that joins `junction`'s beams with `joint_class`, ahead of every category and topology rule."""
    from timber_design.workflow import DirectRule

    if len(junction.beams) != joint_class.MAX_ELEMENT_COUNT:
        raise ValueError("{} joins {} beams, but junction {} has {}.".format(joint_class.__name__, joint_class.MAX_ELEMENT_COUNT, junction.index, len(junction.beams)))
    # Bind beams by role name where the constructor still uses the exported names; by position otherwise.
    names = inspect.getfullargspec(joint_class.__init__).args[1 : 1 + len(junction.beams)]
    beam_by_role = dict(zip(junction.roles, junction.beams))
    beams = [beam_by_role[n] for n in names] if set(names) == set(beam_by_role) else list(junction.beams)
    return DirectRule(joint_class, beams, **params)


def watch(component, folder, names):
    """Re-solve `component` whenever one of the modules `names` is saved in `folder`."""
    import Grasshopper
    import Rhino
    import scriptcontext
    import System

    key = "junction_lib.watch.{}".format(component.InstanceGuid)
    entry = scriptcontext.sticky.get(key)
    if entry is None or entry["folder"] != folder:
        if entry is not None:
            entry["watcher"].Dispose()
        entry = {"folder": folder, "names": set()}

        def expire(doc):
            component.ExpireSolution(False)

        def on_ui_thread():
            document = component.OnPingDocument()
            if document is not None:
                document.ScheduleSolution(300, Grasshopper.Kernel.GH_Document.GH_ScheduleDelegate(expire))

        def on_changed(sender, args):
            if os.path.basename(args.FullPath) in entry["names"]:
                Rhino.RhinoApp.InvokeOnUiThread(System.Action(on_ui_thread))

        watcher = System.IO.FileSystemWatcher(folder, "*.py")
        watcher.NotifyFilter = System.IO.NotifyFilters.LastWrite
        watcher.Changed += on_changed
        watcher.EnableRaisingEvents = True
        entry["watcher"] = watcher
        scriptcontext.sticky[key] = entry
    entry["names"] = set(n + ".py" for n in names)
