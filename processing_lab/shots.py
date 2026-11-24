"""The pictures of docs/b-processings.md: every processing of the lab on a beam, modelled by compas_timber.

    uv run --with pyvista processing_lab/shots.py   # from the repo root

Cyan is what the processing leaves, the dashed outline is the beam it was cut from.
"""

import os

import numpy as np
import pyvista as pv
from compas.geometry import Frame
from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Transformation
from compas_timber.elements import Beam
from server import HERE
from server import VARIANTS
from toolpath_lib import marking_lines
from toolpath_lib import toolpaths

SHOTS = os.path.join(HERE, "..", "docs", "assets", "shots", "processings")
LENGTH, WIDTH, HEIGHT = 300.0, 80.0, 120.0
# the variant that shows best, where it isn't the first
SHOWN = {"Mortise": 1, "House": 1, "HouseMortise": 1, "JackRafterCut": 3, "Slot": 1, "HipValleyRafterNotch": 2, "TriangleCut": 1, "Marking": 2, "Chamfer": 2, "Planing": 2, "FreeContour": 1}
FROM_BELOW = ("StepJoint",)  # cut from the opposite side
GROUND, WOOD, CUT, EDGE, BLANK = "#1d0710", "#e6d3c8", "#4fd6e0", "#19040b", "#8f6571"


def lines(polylines):
    return pv.merge([pv.lines_from_points(np.array(p, dtype=float)) for p in polylines])


def shoot(name):
    _, make = VARIANTS[name][SHOWN.get(name, 0)]
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(LENGTH, 0, 0)), width=WIDTH, height=HEIGHT)
    side_width, side_height = beam.get_dimensions_relative_to_side(0)
    processing = make(LENGTH, side_width, side_height)
    beam.add_features(processing)
    assert any(t.frames for t in toolpaths(beam)), f"{name} has no toolpath"

    geometry = beam.compute_modelgeometry()
    assert not beam.debug_info, f"{name}: {beam.debug_info}"
    up = Transformation.from_frame_to_frame(beam.ref_sides[0], Frame.worldXY())  # side 0 up, its reference corner at the origin
    mesh, edges = geometry.transformed(up).to_tesselation()
    points, faces = (np.array(a) for a in mesh.to_vertices_and_faces(triangulated=True))

    # a triangle lying in a side of the blank is the beam's own, any other was made by the processing
    low, high = np.array([0.0, 0.0, -side_height]), np.array([LENGTH, side_width, 0.0])
    corners = points[faces]
    own = np.any([np.all(np.abs(corners - bound) < 1e-3, axis=1) for bound in (low, high)], axis=(0, 2))
    engraved = name == "Marking"  # leaves the geometry as it is
    assert own.all() == engraved, f"{name}: the beam is unchanged"

    plotter = pv.Plotter(off_screen=True, window_size=(1000, 560))
    plotter.set_background(GROUND)
    for which, color in ((own, WOOD), (~own, CUT)):
        if which.any():
            triangles = np.insert(faces[which], 0, 3, axis=1)
            plotter.add_mesh(pv.PolyData(points, triangles), color=color, ambient=0.35, diffuse=0.65)
    plotter.add_mesh(lines([p.points for p in edges]), color=EDGE, line_width=2)
    for i, j in ((a, b) for a in range(8) for b in range(a) if a ^ b in (1, 2, 4)):
        start, end = (np.where([k & 1, k & 2, k & 4], high, low) for k in (i, j))
        dashes = np.linspace(start, end, 2 * int(np.linalg.norm(end - start) // 10))
        plotter.add_mesh(pv.line_segments_from_points(dashes), color=BLANK, line_width=1.5)
    if engraved:
        plotter.add_mesh(lines([[p.transformed(up) + [0, 0, 0.5] for p in line] for line in marking_lines(beam, processing)]), color=CUT, line_width=6)

    plotter.enable_parallel_projection()
    center = (low + high) / 2
    plotter.camera_position = [center + np.array([750.0, -1000.0, -600.0 if name in FROM_BELOW else 1050.0]), center, (0, 0, 1)]
    plotter.reset_camera()
    plotter.camera.zoom(1.25)
    plotter.enable_anti_aliasing("ssaa")
    plotter.screenshot(os.path.join(SHOTS, f"{name}.png"))
    plotter.close()


if __name__ == "__main__":
    os.makedirs(SHOTS, exist_ok=True)
    for name in VARIANTS:
        shoot(name)
        print(name)
