"""Toolpaths: from a beam's BTLx processings to frames for the milling bit.

Every frame is a position of the bit's tip, its Z axis pointing into the
material. What gets a toolpath today:

- `Lap`: a pocket, cleared in zig-zag lines, layer by layer.

Cuts (`JackRafterCut`, `DoubleCut`) are listed as saw cuts: the 6 mm bit can't
reach through a beam. Anything else is listed as "no toolpath yet" - that list
is the honest answer to "can we mill this joint?".

Adapted from the CAADRIA 2026 workshop's `lib_extra_toolpath_v2.py`.
Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import math

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Polyhedron
from compas.geometry import Vector
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap
from compas_timber.fabrication import LapProxy

BIT_DIAMETER = 6.0  # mm, the bit on site
SAW_CUTS = (JackRafterCut, DoubleCut)


class Toolpath(object):
    """The frames that mill one processing, or why there are none.

    Attributes
    ----------
    processing : BTLxProcessing
    frames : list[Frame]
        Bit tip positions in beam coordinates, Z into the material. Empty if not milled.
    ref_side_index : int
        The side the bit comes from; this side faces up on the table.
    note : str
        What happens to this processing.
    """

    def __init__(self, processing, frames, ref_side_index, note):
        self.processing = processing
        self.frames = frames
        self.ref_side_index = ref_side_index
        self.note = note

    def __repr__(self):
        return "{}: {}".format(type(self.processing).__name__, self.note)

    def ToString(self):
        return repr(self)


def toolpaths(beam, features=None, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """A `Toolpath` for every processing on `beam` (or only those in `features`), in the order they were added."""
    result = []
    for feature in beam.features:
        if features is not None and not any(feature is f for f in features):
            continue
        processing = _unproxified(feature)
        side = getattr(processing, "ref_side_index", 0) or 0
        if isinstance(processing, Lap):
            frames = lap_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
            note = "pocket, {} frames".format(len(frames))
        elif isinstance(processing, SAW_CUTS):
            frames, note = [], "saw cut"
        else:
            frames, note = [], "no toolpath for this processing yet"
        result.append(Toolpath(processing, frames, side, note))
    return result


def _unproxified(feature):
    """The processing behind a proxy (joints add proxies; only the real thing has BTLx parameters)."""
    if not hasattr(feature, "unproxified"):
        return feature
    try:
        return feature.unproxified()
    except AttributeError:
        if not isinstance(feature, LapProxy):
            raise
        # shortcut: compas_timber@9a3e637 asks a Brep face for `.nurbssurface`, which compas_brep 0.1.4 lacks;
        # the lap volume is rebuilt as a Polyhedron instead. Remove when compas_timber is bumped.
        volume = feature.volume.transformed(feature.beam.modeltransformation)
        vertices, faces = [], []
        for face in volume.faces:
            points = list(face.to_polygon().points)
            if face.is_reversed:
                points.reverse()
            faces.append(list(range(len(vertices), len(vertices) + len(points))))
            vertices.extend(points)
        return Lap.from_volume_and_beam(Polyhedron(vertices, faces), feature.beam, feature.machining_limits, feature.ref_side_index)


def lap_toolpath(beam, lap, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear a `Lap` pocket in zig-zag lines, one layer per `stepdown`.

    The pocket's frame comes straight from the Lap's BTLx parameters: it
    starts at (start_x, start_y) on the reference side, runs `length` along
    the start frame's -normal, `width` along its y axis and `depth` into the
    beam. Where the pocket is open at its sides, the bit runs past them by a
    radius so the walls come out clean.

    Returns
    -------
    list[Frame]
        Bit tip positions, approach and retract included.
    """
    radius = bit_diameter / 2.0
    stepover = bit_diameter * 0.6

    start_frame = lap._start_frame_from_params_and_beam(beam)
    side = beam.ref_sides[lap.ref_side_index]
    along = Vector(*(-start_frame.normal))  # length direction
    across = Vector(*start_frame.yaxis)  # width direction
    down = Vector(*(-side.normal))  # into the beam

    usable_length = lap.length - bit_diameter
    if usable_length < 0:
        raise ValueError("A {:.0f} mm bit doesn't fit in a {:.1f} mm long lap.".format(bit_diameter, lap.length))
    side_width = beam.get_dimensions_relative_to_side(lap.ref_side_index)[0]
    open_across = lap.width >= side_width - 1e-3  # the lap runs through the beam's width
    overrun = radius + 1.0 if open_across else 0.0
    line_start, line_end = radius - overrun, lap.width - radius + overrun

    lines = max(1, int(math.ceil(usable_length / stepover)) + 1)
    spacing = usable_length / max(lines - 1, 1)
    layers = max(1, int(math.ceil(lap.depth / stepdown)))

    xaxis = along
    yaxis = Vector(*down.cross(along))
    origin = Point(*start_frame.point) + along * radius

    def frame(at_along, at_across, at_depth):
        return Frame(origin + along * at_along + across * at_across + down * at_depth, xaxis, yaxis)

    frames = [frame(0.0, line_start, -safe_height)]
    for layer in range(1, layers + 1):
        depth = lap.depth * layer / layers
        for line in range(lines):
            a, b = (line_start, line_end) if line % 2 == 0 else (line_end, line_start)
            frames.append(frame(spacing * line, a, depth))
            frames.append(frame(spacing * line, b, depth))
        frames.append(frame(spacing * (lines - 1), b, -safe_height))  # retract
        if layer < layers:
            frames.append(frame(0.0, line_start, -safe_height))
    return frames


if __name__ == "__main__":
    # Self-check: every cutting move of a through-lap stays inside the pocket.
    from compas.geometry import Line
    from compas_timber.elements import Beam

    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=60, height=120)
    lap = Lap(start_x=200.0, length=80.0, width=60.0, depth=30.0, ref_side_index=0)
    beam.add_features(lap)
    (toolpath,) = toolpaths(beam)
    side, start = beam.ref_sides[0], lap._start_frame_from_params_and_beam(beam)
    for f in toolpath.frames:
        depth = -side.to_local_coordinates(f.point)[2]
        if depth > 0:
            assert depth <= lap.depth + 1e-6
            assert BIT_DIAMETER / 2 - 1e-6 <= (f.point - start.point).dot(-start.normal) <= lap.length - BIT_DIAMETER / 2 + 1e-6
    print("Lap toolpath stays inside the pocket ({} frames).".format(len(toolpath.frames)))
