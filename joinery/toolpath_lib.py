"""Toolpaths: from a beam's BTLx processings to frames for the milling bit.

Every frame is a position of the bit's tip, its Z axis pointing into the
material. What gets a toolpath today:

- `Lap`: a pocket, cleared in zig-zag lines, layer by layer.
- `StepJointNotch`: cleared with the bit square to its seat, the long face
  the strut sits on.

Cuts (`JackRafterCut`, `DoubleCut`, `StepJoint`) are listed as saw cuts, with
what the saw needs: the 6 mm bit can't reach through a beam. Anything else is listed as "no toolpath
yet" - that list is the honest answer to "can we mill this joint?".

Adapted from the CAADRIA 2026 workshop's `lib_extra_toolpath_v2.py`.
Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import math

from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import intersection_line_plane
from compas.geometry import intersection_plane_plane_plane
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap
from compas_timber.fabrication import StepJoint
from compas_timber.fabrication import StepJointNotch
from compas_timber.fabrication import StepShapeType

BIT_DIAMETER = 6.0  # mm, the bit on site
CLEARANCE = 5.0  # mm between the bit and a beam edge it cuts through, so it goes in and out of the wood in the air
SAW_CUTS = (JackRafterCut, DoubleCut, StepJoint)


class Toolpath:
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
        return f"{type(self.processing).__name__}: {self.note}"

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
            note = f"pocket, {len(frames)} frames"
        elif isinstance(processing, StepJointNotch) and processing.step_shape == StepShapeType.STEP:
            frames = step_notch_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
            note = f"step notch, {len(frames)} frames"
        elif isinstance(processing, SAW_CUTS):
            frames, note = [], saw_cut_note(beam, processing)
        else:
            frames, note = [], "no toolpath for this processing yet"
        result.append(Toolpath(processing, frames, side, note))
    return result


def saw_cut_note(beam, cut):
    """Where to saw, for each plane of `cut`: where it crosses the beam's axis,
    measured from the start of the timber, and its angle from a square cut as
    seen on side 0 and on side 1."""
    planes = cut.planes_from_params_and_beam(beam) if hasattr(cut, "planes_from_params_and_beam") else [cut.plane_from_params_and_beam(beam)]
    start = beam.ref_sides[0]
    lines = []
    for plane in planes:
        at = start.to_local_coordinates(Point(*intersection_line_plane(beam.centerline, plane)))[0]
        on_0, on_1 = (_angle_from_square(plane, beam.ref_sides[i]) for i in (0, 1))
        lines.append(f"at {at:.1f} mm, {on_0:.1f}° on side 0, {on_1:.1f}° on side 1")
    return "saw " + "; ".join(lines)


def _angle_from_square(plane, side):
    """The angle between the cut's line on `side` and a square cut across it."""
    cut_line = Vector(*plane.normal).cross(side.normal)
    angle = math.degrees(cut_line.angle(side.yaxis))
    return min(angle, 180.0 - angle)


def _unproxified(feature):
    """The processing behind a proxy (joints add proxies; only the real thing has BTLx parameters)."""
    return feature.unproxified() if hasattr(feature, "unproxified") else feature


def lap_toolpath(beam, lap, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear a `Lap` pocket in zig-zag lines, one layer per `stepdown`.

    The pocket's frame comes straight from the Lap's BTLx parameters: it
    starts at (start_x, start_y) on the reference side, runs `length` along
    the start frame's -normal, `width` along its y axis and `depth` into the
    beam. Where the pocket is open at its sides, the bit runs all the way out of
    the wood, `CLEARANCE` past them.

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
        raise ValueError(f"A {bit_diameter:.0f} mm bit doesn't fit in a {lap.length:.1f} mm long lap.")
    side_width = beam.get_dimensions_relative_to_side(lap.ref_side_index)[0]
    open_across = lap.width >= side_width - 1e-3  # the lap runs through the beam's width
    if open_across:
        line_start, line_end = -radius - CLEARANCE, lap.width + radius + CLEARANCE
    else:
        line_start, line_end = radius, lap.width - radius

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


def step_notch_toolpath(beam, notch, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear a step `StepJointNotch` with the bit square to its seat, the long face the strut sits on.

    The notch is a V of two sloped faces: the long, flat seat and the short,
    steep step. It is cleared in layers parallel to the seat, ending on the
    seat itself, each layer in zig-zag lines across the notch from the step
    out to the beam's face.

    Returns
    -------
    list[Frame]
        Bit tip positions, approach and retract included.
    """
    radius = bit_diameter / 2.0
    stepover = bit_diameter * 0.6

    side = beam.ref_sides[notch.ref_side_index]
    across = Vector(*side.yaxis)
    top_ends = [Point(*side.point) + Vector(*side.xaxis) * x + across * notch.start_y for x in (notch.start_x, notch.start_x + notch.displacement_end)]
    planes = notch.planes_from_params_and_beam(beam)
    bottom = Point(*intersection_plane_plane_plane(planes[0], planes[1], Plane(top_ends[0], across)))
    seat_end, step_end = sorted(top_ends, key=lambda end: (end - bottom).length, reverse=True)

    up = (seat_end - bottom).unitized()
    if up.dot(step_end - bottom) > 0:
        raise ValueError("The notch's faces meet at less than 90 degrees: a bit square to the seat would cut into the step.")
    out = (step_end - bottom) - up * (step_end - bottom).dot(up)
    thickness = out.length  # how far the step's top edge stands off the seat
    out.unitize()
    into = -out
    seat_length = (seat_end - bottom).length
    step_along = (step_end - bottom).dot(up)

    side_width = beam.get_dimensions_relative_to_side(notch.ref_side_index)[0]
    open_across = notch.notch_width >= side_width - 1e-3
    if open_across:
        line_start, line_end = -radius - CLEARANCE, notch.notch_width + radius + CLEARANCE
    else:
        line_start, line_end = radius, notch.notch_width - radius

    def frame(along, over, at_across):
        return Frame(bottom + up * along + out * over + across * at_across, into.cross(across), -across)

    frames = []
    layers = max(1, int(math.ceil(thickness / stepdown)))
    for layer in range(1, layers + 1):
        over = thickness * (layers - layer) / layers
        first = over / thickness * step_along + radius
        last = seat_length + over / thickness * (step_along - seat_length) + radius
        if last <= first:
            continue
        lines = max(2, int(math.ceil((last - first) / stepover)) + 1)
        positions = [first + (last - first) * i / (lines - 1) for i in range(lines)]
        frames.append(frame(positions[0], over + safe_height, line_start))
        for line, along in enumerate(positions):
            a, b = (line_start, line_end) if line % 2 == 0 else (line_end, line_start)
            frames.append(frame(along, over, a))
            frames.append(frame(along, over, b))
        frames.append(frame(positions[-1], over + safe_height, b))
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
    print(f"Lap toolpath stays inside the pocket ({len(toolpath.frames)} frames).")

    # Self-check: every cutting move of a step notch stays inside the notch: above both of its faces, never deeper than its step.
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=100, height=100)
    notch = StepJointNotch(
        start_x=200.0,
        start_y=10.0,
        strut_inclination=105.0,
        notch_limited=True,
        notch_width=80.0,
        step_depth=15.0,
        heel_depth=0.0,
        strut_height=160.0,
        step_shape=StepShapeType.STEP,
    )
    beam.add_features(notch)
    (toolpath,) = toolpaths(beam)
    side = beam.ref_sides[0]
    opening = side.point + side.xaxis * (notch.start_x + notch.displacement_end / 2) + side.yaxis * (notch.start_y + notch.notch_width / 2)
    for f in toolpath.frames:
        x, y, z = side.to_local_coordinates(f.point)
        if -z > 1e-6:
            assert -z <= notch.step_depth + 1e-6
            assert notch.start_y + BIT_DIAMETER / 2 - 1e-6 <= y <= notch.start_y + notch.notch_width - BIT_DIAMETER / 2 + 1e-6
            for plane in notch.planes_from_params_and_beam(beam):
                assert (f.point - plane.point).dot(plane.normal) * (opening - plane.point).dot(plane.normal) >= -1e-6
    print(f"Step notch toolpath stays inside the notch ({len(toolpath.frames)} frames).")

    # Self-check: a saw cut's note reads back the cut's own parameters.
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=160)
    assert saw_cut_note(beam, JackRafterCut(start_x=500.0, angle=90.0, inclination=90.0)) == "saw at 500.0 mm, 0.0° on side 0, 0.0° on side 1"
    assert saw_cut_note(beam, JackRafterCut(start_x=500.0, angle=60.0, inclination=90.0)) == "saw at 523.1 mm, 30.0° on side 0, 0.0° on side 1"
    assert saw_cut_note(beam, JackRafterCut(start_x=500.0, angle=90.0, inclination=70.0)) == "saw at 529.1 mm, 0.0° on side 0, 20.0° on side 1"
    print("Saw cut notes read back the cuts' angles and positions.")
