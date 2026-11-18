"""Processing lab: pick a processing, see its toolpath on a beam and the IRB2600 following it.

A helper for writing toolpaths, not workshop material. The toolpaths come from
`joinery/toolpath_lib.py`, the robot, its spindle and the planning from
`joinery/robot_lib.py`: what you see here is what Grasshopper gets.

    uv run processing_lab/server.py   # from the repo root
    open http://localhost:8000

To support another processing: write its toolpath in `toolpath_lib.py`, then
add a few variants of it to `VARIANTS` below.
"""

import faulthandler
import inspect
import json
import os
import sys
import traceback
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "joinery"))

import compas_timber.fabrication as ctf  # noqa: E402
import wjim_lib  # noqa: E402
from compas.geometry import Line  # noqa: E402
from compas.geometry import Point  # noqa: E402
from compas.geometry import Polyline  # noqa: E402
from compas.geometry import Scale  # noqa: E402
from compas.geometry import Transformation  # noqa: E402
from compas_robots import Configuration  # noqa: E402
from compas_timber.elements import Beam  # noqa: E402
from compas_timber.fabrication import BirdsMouth  # noqa: E402
from compas_timber.fabrication import BTLxProcessing  # noqa: E402
from compas_timber.fabrication import Contour  # noqa: E402
from compas_timber.fabrication import DoubleCut  # noqa: E402
from compas_timber.fabrication import Drilling  # noqa: E402
from compas_timber.fabrication import FreeContour  # noqa: E402
from compas_timber.fabrication import JackRafterCut  # noqa: E402
from compas_timber.fabrication import Lap  # noqa: E402
from compas_timber.fabrication import LongitudinalCut  # noqa: E402
from compas_timber.fabrication import Mortise  # noqa: E402
from compas_timber.fabrication import Pocket  # noqa: E402
from compas_timber.fabrication import Slot  # noqa: E402
from compas_timber.fabrication import StepJointNotch  # noqa: E402
from compas_timber.fabrication import OrientationType  # noqa: E402
from compas_timber.fabrication import StepShapeType  # noqa: E402
from compas_timber.fabrication import Tenon  # noqa: E402
from robot_lib import MM_PER_M  # noqa: E402
from robot_lib import MillingCell  # noqa: E402
from toolpath_lib import toolpaths  # noqa: E402
from wjim_lib import Chamfer  # noqa: E402
from wjim_lib import House  # noqa: E402
from wjim_lib import HouseMortise  # noqa: E402
from wjim_lib import LogHouseHalfLap  # noqa: E402
from wjim_lib import Marking  # noqa: E402
from wjim_lib import MillContour  # noqa: E402
from wjim_lib import Planing  # noqa: E402

PORT = 8000
CELL = MillingCell()


def _sq(**kwargs):
    return Mortise(shape="square", **kwargs)


def _outline(L, w, corners):
    """A closed outline on side 0, its corners around the middle of the side, counterclockwise seen from outside."""
    return Polyline([[0.5 * L + x, 0.5 * w + y, 0.0] for x, y in corners + corners[:1]])


def _tenon(**kwargs):
    kwargs = {"orientation": OrientationType.END, "angle": 90.0, "inclination": 90.0, "rotation": 90.0, "shape": "square", **kwargs}
    return Tenon(**kwargs)


# name -> variants, each a label and a function of (beam length, width and height of side 0) to the processing
VARIANTS = {
    "Lap": [
        ("through, 80 long, 30 deep", lambda L, w, h: Lap(start_x=0.3 * L, length=80.0, width=w, depth=30.0)),
        ("through, 40 long, 15 deep", lambda L, w, h: Lap(start_x=0.3 * L, length=40.0, width=w, depth=15.0)),
    ],
    "StepJointNotch": [
        (
            "step, 15 deep",
            lambda L, w, h: StepJointNotch(
                start_x=0.3 * L,
                start_y=10.0,
                strut_inclination=105.0,
                notch_limited=True,
                notch_width=w - 20.0,
                step_depth=15.0,
                heel_depth=0.0,
                strut_height=160.0,
                step_shape=StepShapeType.STEP,
            ),
        ),
    ],
    "Mortise": [
        ("60 x 20, 40 deep", lambda L, w, h: _sq(start_x=0.5 * L, start_y=w / 2, length=60.0, width=20.0, depth=40.0)),
        ("60 x 20, 40 deep, turned 30°", lambda L, w, h: _sq(start_x=0.5 * L, start_y=w / 2, angle=30.0, length=60.0, width=20.0, depth=40.0)),
        ("40 x 8, 30 deep", lambda L, w, h: _sq(start_x=0.5 * L, start_y=w / 2, length=40.0, width=8.0, depth=30.0)),
        ("40 x 6 (bit wide), 20 deep", lambda L, w, h: _sq(start_x=0.5 * L, start_y=w / 2, length=40.0, width=6.0, depth=20.0)),
        ("40 x 4 (too narrow)", lambda L, w, h: _sq(start_x=0.5 * L, start_y=w / 2, length=40.0, width=4.0, depth=20.0)),
    ],
    "Tenon": [
        ("60 x 20, 40 long, centred (fits the 60 x 20 mortise)", lambda L, w, h: _tenon(start_x=L - 40.0, start_y=w / 2, start_depth=(h - 60.0) / 2, length=60.0, width=20.0, height=40.0)),
        ("60 x 20, 40 long, 20 mm of beam past its tip", lambda L, w, h: _tenon(start_x=L - 60.0, start_y=w / 2, start_depth=(h - 60.0) / 2, length=60.0, width=20.0, height=40.0)),
        ("80 x 30, 50 long, near the top", lambda L, w, h: _tenon(start_x=L - 50.0, start_y=w / 2, start_depth=10.0, length=min(80.0, h - 20.0), width=30.0, height=50.0)),
        ("full height, 30 wide, flush with side 1", lambda L, w, h: _tenon(start_x=L - 40.0, start_y=15.0, start_depth=0.0, length_limited_top=False, length_limited_bottom=False, length=h, width=30.0, height=40.0)),
        ("at the start of the beam", lambda L, w, h: _tenon(orientation=OrientationType.START, start_x=40.0, start_y=w / 2, start_depth=(h - 60.0) / 2, length=60.0, width=20.0, height=40.0)),
    ],
    "House": [
        ("80 x 60, 20 long, centred", lambda L, w, h: House(orientation=OrientationType.END, start_x=L - 20.0, start_y=w / 2, start_depth=(h - 80.0) / 2, length=80.0, width=min(60.0, w - 10), height=20.0, shape="square")),
        (
            "80 x 60 x 20, tenon 50 x 20 x 30 on it",
            lambda L, w, h: House(
                orientation=OrientationType.END,
                start_x=L - 50.0,
                start_y=w / 2,
                start_depth=(h - 80.0) / 2,
                length=80.0,
                width=min(60.0, w - 10),
                height=20.0,
                shape="square",
                subprocessings=[_tenon(start_x=L - 30.0, start_y=w / 2, start_depth=(h - 50.0) / 2, length=50.0, width=20.0, height=30.0)],
            ),
        ),
    ],
    "JackRafterCut": [
        ("square, 40 from the end", lambda L, w, h: JackRafterCut(orientation=OrientationType.END, start_x=L - 40.0)),
        ("angle 60°", lambda L, w, h: JackRafterCut(orientation=OrientationType.END, start_x=L - 60.0, angle=60.0)),
        ("inclination 70° (from the opposite side)", lambda L, w, h: JackRafterCut(orientation=OrientationType.END, start_x=L - 60.0, inclination=70.0)),
        ("inclination 110°", lambda L, w, h: JackRafterCut(orientation=OrientationType.END, start_x=L - 60.0, inclination=110.0)),
    ],
    "DoubleCut": [
        ("pointed end, 45° + 135°", lambda L, w, h: DoubleCut(orientation=OrientationType.END, start_x=L - w / 2, start_y=w / 2, angle_1=135.0, angle_2=45.0)),
        ("V notch at the end, 45° + 135°", lambda L, w, h: DoubleCut(orientation=OrientationType.END, start_x=L - w / 2, start_y=w / 2, angle_1=45.0, angle_2=135.0)),
    ],
    "LongitudinalCut": [
        ("edge off at 45°, 20 in", lambda L, w, h: LongitudinalCut(start_y=20.0, inclination=45.0)),
        ("limited (not milled)", lambda L, w, h: LongitudinalCut(start_x=0.3 * L, start_y=20.0, inclination=45.0, start_limited=True, end_limited=True, length=200.0)),
    ],
    "Slot": [
        ("10 thick, 50 deep, across the beam", lambda L, w, h: Slot(start_x=0.5 * L, start_y=0.0, length=w, depth=50.0, thickness=10.0)),
        ("6 thick (bit wide), 30 deep, angle 60°", lambda L, w, h: Slot(start_x=0.5 * L, start_y=0.0, angle=60.0, length=w, depth=30.0, thickness=6.0)),
        ("4 thick (too narrow)", lambda L, w, h: Slot(start_x=0.5 * L, start_y=0.0, length=w, depth=30.0, thickness=4.0)),
    ],
    "HouseMortise": [
        ("house 80 x 60, 10 deep", lambda L, w, h: HouseMortise(start_x=0.5 * L, start_y=w / 2, length=80.0, width=min(60.0, w - 10), depth=10.0, shape="square")),
        (
            "house 80 x 60 x 10, mortise 50 x 20 x 30 in it",
            lambda L, w, h: HouseMortise(
                start_x=0.5 * L,
                start_y=w / 2,
                length=80.0,
                width=min(60.0, w - 10),
                depth=10.0,
                shape="square",
                subprocessings=[_sq(start_x=0.5 * L + 15.0, start_y=w / 2, start_depth=10.0, length=50.0, width=20.0, depth=30.0)],
            ),
        ),
    ],
    "Drilling": [
        ("Ø20, 40 deep", lambda L, w, h: Drilling(start_x=0.5 * L, start_y=w / 2, depth_limited=True, depth=40.0, diameter=20.0)),
        ("Ø6 (bit wide), 30 deep", lambda L, w, h: Drilling(start_x=0.5 * L, start_y=w / 2, depth_limited=True, depth=30.0, diameter=6.0)),
        ("Ø12, through", lambda L, w, h: Drilling(start_x=0.5 * L, start_y=w / 2, depth_limited=False, diameter=12.0)),
        ("Ø16, inclined 60°, 40 deep", lambda L, w, h: Drilling(start_x=0.5 * L, start_y=w / 2, inclination=60.0, depth_limited=True, depth=40.0, diameter=16.0)),
        ("Ø10, through, angle 45°, inclined 70°", lambda L, w, h: Drilling(start_x=0.5 * L, start_y=w / 2, angle=45.0, inclination=70.0, diameter=10.0)),
    ],
    "Marking": [
        ("single line across", lambda L, w, h: Marking(start_x=0.5 * L, start_y=5.0, angle=90.0, length_limited=True, length=w - 10.0)),
        ("double, 40 apart", lambda L, w, h: Marking(start_x=0.5 * L, start_y=5.0, angle=90.0, length_limited=True, length=w - 10.0, width=40.0, style="double")),
        ("square 50 x 30, turned 30°", lambda L, w, h: Marking(start_x=0.5 * L, start_y=25.0, angle=120.0, length_limited=True, length=50.0, width=30.0, style="square")),
        ("single, whole side, 45°", lambda L, w, h: Marking(start_x=0.5 * L, start_y=w / 2, angle=45.0, length_limited=False)),
    ],
    "Chamfer": [
        ("edge 12, through, 10 mm", lambda L, w, h: Chamfer(depth=10.0, chamfer_edge_23=False, chamfer_edge_34=False, chamfer_edge_41=False)),
        ("edges 12 + 41, through, 6 mm", lambda L, w, h: Chamfer(depth=6.0, chamfer_edge_23=False, chamfer_edge_34=False)),
        (
            "edge 12, 200 long, 10 mm",
            lambda L, w, h: Chamfer(
                start_x=0.5 * L - 100.0, length=200.0, start_limited=True, end_limited=True, depth=10.0, chamfer_edge_23=False, chamfer_edge_34=False, chamfer_edge_41=False
            ),
        ),
        ("all four edges, 5 mm", lambda L, w, h: Chamfer(depth=5.0)),
    ],
    "Pocket": [
        ("100 x 40, 25 deep", lambda L, w, h: Pocket(start_x=0.4 * L, start_y=20.0, start_depth=25.0, length=100.0, width=40.0)),
        ("100 long, through the width, 20 deep", lambda L, w, h: Pocket(start_x=0.4 * L, start_y=0.0, start_depth=20.0, length=100.0, width=w)),
        ("start side tilted 60° (undercut)", lambda L, w, h: Pocket(start_x=0.4 * L, start_y=20.0, start_depth=25.0, length=100.0, width=40.0, tilt_start_side=60.0)),
    ],
    "BirdsMouth": [
        ("seat 45°, heel 135°, 30 deep", lambda L, w, h: BirdsMouth(start_x=0.5 * L, start_depth=30.0, depth=30.0, width=w)),
        ("seat 60°, heel 150°, 25 deep", lambda L, w, h: BirdsMouth(start_x=0.5 * L, start_depth=25.0, depth=25.0, width=w, inclination_1=60.0, inclination_2=150.0)),
        ("angle 70°, 20 deep, orientation end", lambda L, w, h: BirdsMouth(orientation=OrientationType.END, start_x=0.5 * L, start_depth=20.0, depth=20.0, width=w, angle=70.0)),
    ],
    "LogHouseHalfLap": [
        ("80 wide, 30 deep from both sides", lambda L, w, h: LogHouseHalfLap(start_x=0.5 * L, length=80.0, depth_ref_side=30.0, depth_opp_side=30.0)),
        ("80 wide, at 60°, 25 + 35 deep", lambda L, w, h: LogHouseHalfLap(start_x=0.5 * L, angle=60.0, length=80.0, depth_ref_side=25.0, depth_opp_side=35.0)),
        ("only from side 0, 40 deep", lambda L, w, h: LogHouseHalfLap(start_x=0.5 * L, length=60.0, depth_ref_side=40.0, depth_opp_side=0.0)),
        ("at the end of the beam", lambda L, w, h: LogHouseHalfLap(orientation=OrientationType.END, start_x=L - 30.0, length=80.0, depth_ref_side=30.0, depth_opp_side=30.0)),
    ],
    "Planing": [
        ("side 0, 150 long, 3 mm", lambda L, w, h: Planing(start_x=0.4 * L, length=150.0, depth=3.0, start_limited=True, end_limited=True, plane_side_2=False, plane_side_3=False, plane_side_4=False)),
        ("sides 0 and 2, 100 long, 8 mm", lambda L, w, h: Planing(start_x=0.4 * L, length=100.0, depth=8.0, start_limited=True, end_limited=True, plane_side_2=False, plane_side_4=False)),
        ("side 1, the last 100 mm, 5 mm", lambda L, w, h: Planing(start_x=L - 100.0, length=100.0, depth=5.0, start_limited=True, plane_side_1=False, plane_side_3=False, plane_side_4=False)),
    ],
    "FreeContour": [
        ("rectangle 100 x 40, inside removed, 20 deep", lambda L, w, h: FreeContour(Contour(_outline(L, w, [(-50, -20), (50, -20), (50, 20), (-50, 20)]), 20.0), counter_sink=True)),
        ("L shape, inside removed, 15 deep (outline only)", lambda L, w, h: FreeContour(Contour(_outline(L, w, [(-50, -25), (50, -25), (50, 0), (0, 0), (0, 25), (-50, 25)]), 15.0), counter_sink=True)),
        ("triangle, through, inside comes loose", lambda L, w, h: FreeContour(Contour(_outline(L, w, [(-30, -20), (30, -20), (0, 25)]), h), counter_sink=True, depth_bounded=False)),
        ("open line across the side, tool centred, 10 deep", lambda L, w, h: FreeContour(Contour(Polyline([[0.5 * L - 30, 0, 0], [0.5 * L + 30, w, 0]]), 10.0), tool_position="center")),
    ],
    "MillContour": [
        ("square 50 x 50, inside removed, 30 deep", lambda L, w, h: MillContour(Contour(_outline(L, w, [(-25, -25), (25, -25), (25, 25), (-25, 25)]), 30.0), counter_sink=True, tool_position="left")),
        ("square 50 x 50, tool outside, 10 deep", lambda L, w, h: MillContour(Contour(_outline(L, w, [(-25, -25), (25, -25), (25, 25), (-25, 25)]), 10.0), tool_position="right")),
        ("open U from the edge, tool inside it, 12 deep", lambda L, w, h: MillContour(Contour(Polyline([[0.5 * L - 30, 0, 0], [0.5 * L - 30, 30, 0], [0.5 * L + 30, 30, 0], [0.5 * L + 30, 0, 0]]), 12.0), tool_position="right")),
    ],
}


def catalog():
    """Every processing of compas_timber.fabrication and wjim_lib, and the variants we have for it."""
    result, seen = [], set()
    for module in (ctf, wjim_lib):
        for name, cls in sorted(vars(module).items()):
            if inspect.isclass(cls) and issubclass(cls, BTLxProcessing) and cls is not BTLxProcessing and not inspect.isabstract(cls) and cls not in seen:
                seen.add(cls)
                result.append({"name": name, "package": module.__name__.split(".")[0], "variants": [label for label, _ in VARIANTS.get(name, [])]})
    return result


def _matrix(transformation):
    """Column-major, as three.js `Matrix4.fromArray` reads it."""
    return [round(transformation.matrix[r][c], 4) for c in range(4) for r in range(4)]


def _mesh(mesh, transformation=None):
    if transformation is not None:
        mesh = mesh.transformed(transformation)
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    return {"vertices": [round(x, 3) for v in vertices for x in v], "faces": [i for f in faces for i in f]}


def _in_mm(transformation):
    s = Scale.from_factors([MM_PER_M] * 3)
    return s * transformation * s.inverse()


def robot():
    """The robot's link meshes at zero, in the cell (mm), and the spindle's mesh on the flange."""
    model = CELL.robot
    base = Transformation.from_frame(CELL.robot_frame) * Scale.from_factors([MM_PER_M] * 3)
    links = []
    for link in model.iter_links():
        meshes = [_mesh(m, base * item.init_transformation) for item in link.visual for m in item.geometry.shape.meshes]
        if meshes:
            links.append({"name": link.name, "meshes": meshes})
    tool = CELL.tool.root.visual[0].geometry.shape.meshes[0].scaled(MM_PER_M)
    return {"links": links, "tool": _mesh(tool)}


def run(length, width, height, name, index):
    """The beam with variant `index` of processing `name`, and for every side it is milled from (a setup):
    the beam clamped that side up, the toolpath and the robot's trajectory along it."""
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(length, 0, 0)), width=width, height=height)
    label, make = VARIANTS[name][index]
    processing = make(length, *beam.get_dimensions_relative_to_side(0))
    beam.add_features(processing)
    result = {"label": label, "params": {k: v for k, v in processing.__data__.items() if isinstance(v, (str, int, float, bool, type(None)))}}
    try:
        paths = toolpaths(beam)
        result["note"] = "\n".join(t.note for t in paths)
        paths = [t for t in paths if t.frames]
    except Exception as e:
        paths = []
        result["error"] = f"toolpath: {e}"

    try:
        geometry = beam.compute_modelgeometry()
        result["debug"] = [str(e) for e in beam.debug_info]
    except Exception as e:  # compas_timber can't model some processings (an open FreeContour): show the bare beam
        geometry = beam.compute_elementgeometry(include_features=False).transformed(beam.transformation)
        result["debug"] = [f"no geometry for the processing, the beam is shown without it: {e}"]
    mesh, polylines = geometry.to_tesselation()
    sides = list(dict.fromkeys(t.ref_side_index for t in paths)) or [0]
    result["setups"] = [_setup(beam, mesh, polylines, paths, side) for side in sides]
    return result


def _setup(beam, mesh, polylines, paths, side):
    placement, frames, _, _ = CELL.setup(beam, paths, side)
    result = {
        "side": side,
        "beam": _mesh(mesh, placement),
        "edges": [[round(x, 3) for p in pl.transformed(placement).points for x in p] for pl in polylines],
        "path": [[round(x, 3) for x in f.point] for f in frames],
    }
    if not frames:
        return result
    try:
        trajectory = CELL.plan(frames)
    except Exception as e:
        result["plan_error"] = f"{type(e).__name__}: {e}"
        return result
    model = CELL.robot
    joints = model.get_configurable_joint_names()
    base = Transformation.from_frame(CELL.robot_frame)
    unbase = base.inverse()  # the link meshes are already placed in the cell
    names = [link["name"] for link in robot_links()]
    by_link = {j.child_link.name: j.name for j in model.iter_joints()}
    points = []
    for q in trajectory:
        transformations = model.compute_transformations(Configuration.from_revolute_values(q, joints))
        flange = Transformation.from_frame(CELL.tool0(q))
        points.append(
            {
                "q": [round(v, 5) for v in q],
                "links": [_matrix(base * _in_mm(transformations[by_link[n]]) * unbase) if n in by_link else None for n in names],
                "tool": _matrix(flange),
                "tip": [round(x, 3) for x in CELL.tool_tip(q).point],
            }
        )
    result["trajectory"] = points
    return result


_ROBOT = None


def robot_links():
    global _ROBOT
    _ROBOT = _ROBOT or robot()
    return _ROBOT["links"]


class Handler(BaseHTTPRequestHandler):
    def _send(self, body, content_type="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                return self._send(f.read(), "text/html; charset=utf-8")
        if self.path == "/api/catalog":
            return self._send(catalog())
        if self.path == "/api/robot":
            robot_links()
            return self._send(_ROBOT)
        self.send_error(404)

    def do_POST(self):
        if self.path != "/api/run":
            return self.send_error(404)
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        try:
            body = run(float(request["length"]), float(request["width"]), float(request["height"]), request["name"], int(request["index"]))
        except Exception:
            body = {"error": traceback.format_exc()}
        self._send(body)


if __name__ == "__main__":
    faulthandler.enable()  # a crash in the native planner or OCC leaves a stack trace
    print(f"Processing lab on http://localhost:{PORT}")
    HTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
