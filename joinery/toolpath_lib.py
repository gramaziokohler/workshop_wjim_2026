"""Toolpaths: from a beam's BTLx processings to frames for the milling bit.

Every frame is a position of the bit's tip, its Z axis pointing into the
material. What gets a toolpath today:

- `Lap`: a pocket, cleared in zig-zag lines, layer by layer.
- `StepJointNotch`: cleared with the bit square to its seat, the long face
  the strut sits on.
- `Pocket`, `Mortise`: any convex volume, cleared in zig-zag layers with the bit
  square to the ref side. Inside corners stay round, at the bit's radius.
- `HouseMortise`: a closed pocket, zig-zag along its long side, then the mortise
  nested in it; the corners come out rounded to the bit's radius.
- `Drilling`: a helix down the hole, the core cleared ring by ring.
- `Marking`: lines engraved `MARK_DEPTH` deep.
- `Chamfer`: faced with the bit square to the chamfer, layer by layer, on the
  edges of the side facing up.
- `Tenon`: in two setups, the beam flat on the table: the top half from the
  reference side, the bottom half with the beam flipped onto it. Two `Toolpath`s.

Cuts (`JackRafterCut`, `DoubleCut`, `StepJoint`) are listed as saw cuts, with
what the saw needs: the 6 mm bit can't reach through a beam. Anything else is listed as "no toolpath
yet" - that list is the honest answer to "can we mill this joint?".

Adapted from the CAADRIA 2026 workshop's `lib_extra_toolpath_v2.py`.
Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import math

from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import intersection_line_plane
from compas.geometry import intersection_plane_plane_plane
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import Drilling
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap
from compas_timber.fabrication import Mortise
from compas_timber.fabrication import OrientationType
from compas_timber.fabrication import Pocket
from compas_timber.fabrication import StepJoint
from compas_timber.fabrication import StepJointNotch
from compas_timber.fabrication import StepShapeType
from compas_timber.fabrication import Tenon
from wjim_lib import Chamfer
from wjim_lib import ChamferExitType
from wjim_lib import HouseMortise
from wjim_lib import Marking
from wjim_lib import MarkingStyleType

BIT_DIAMETER = 6.0  # mm, the bit on site
CLEARANCE = 5.0  # mm between the bit and a beam edge it cuts through, so it goes in and out of the wood in the air
MARK_DEPTH = 1.0  # mm, how deep a Marking is engraved
SAW_CUTS = (JackRafterCut, DoubleCut, StepJoint)
CHAMFER_EDGES = ("12", "23", "34", "41")  # Chamfer's edge flags, edge i joins ref sides i and i + 1


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
        elif isinstance(processing, HouseMortise):  # before Mortise, it is one
            frames = house_mortise_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
            note = f"house pocket, {len(frames)} frames"
        elif isinstance(processing, (Pocket, Mortise)):
            frames, note = pocket_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
        elif isinstance(processing, Drilling):
            frames = drilling_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
            note = f"helix, {len(frames)} frames"
        elif isinstance(processing, Marking):
            frames = marking_toolpath(beam, processing, safe_height=safe_height)
            note = f"engraved {MARK_DEPTH:.0f} mm deep with the {bit_diameter:.0f} mm bit, {len(frames)} frames"
        elif isinstance(processing, Chamfer):
            frames = chamfer_toolpath(beam, processing, side, bit_diameter, stepdown, safe_height)
            elsewhere = [name for i, name in enumerate(CHAMFER_EDGES) if getattr(processing, "chamfer_edge_" + name) and side not in (i, (i + 1) % 4)]
            note = f"chamfer facing, {len(frames)} frames" + (f"; edges {', '.join(elsewhere)} need the beam turned" if elsewhere else "")
        elif isinstance(processing, Tenon):
            for from_bottom in (False, True):
                frames, depth = tenon_toolpath(beam, processing, from_bottom, bit_diameter, stepdown, safe_height)
                at = (side + 2) % 4 if from_bottom else side
                how = f"from side {at}, the beam flipped" if from_bottom else f"from side {at}"
                result.append(Toolpath(processing, frames, at, f"tenon {how}, {depth:.0f} mm deep, {len(frames)} frames"))
            continue
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


def _volume(beam, processing):
    """The convex volume `processing` removes, as (vertices, faces)."""
    if isinstance(processing, Pocket):
        volume = processing.volume_from_params_and_element(beam)
        return volume.vertices, volume.faces
    # Mortise: the box its own volume_from_params_and_beam builds, without going through a Brep.
    frame = processing.frame_from_params_and_beam(beam)
    frame.translate((-frame.normal * processing.depth - frame.yaxis * processing.length) * 0.5)
    return Box(processing.width, processing.length, processing.depth, frame).to_vertices_and_faces()


def pocket_toolpath(beam, processing, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear the convex volume of a `Pocket` or `Mortise` with the bit square to its ref side, in zig-zag layers.

    Every face of the volume is a half-space. At each depth, the bit's centre may go where its whole
    disc is inside all of them, there and at the surface (so its shaft doesn't cut an overhang on the
    way down). A face flush with or outside a side of the beam is open: the bit runs past it by `CLEARANCE`.

    Returns
    -------
    tuple
        Bit tip positions, approach and retract included (empty if the bit can't get in from this side),
        and the note: a face that hangs over the volume is an undercut the bit leaves standing.
    """
    radius = bit_diameter / 2.0
    stepover = bit_diameter * 0.6
    side = beam.ref_sides[processing.ref_side_index]
    vertices, faces = _volume(beam, processing)
    local = [Point(*side.to_local_coordinates(Point(*v))) for v in vertices]  # z out of the side, depth = -z
    centre = Point(*[sum(c) / len(local) for c in zip(*local)])
    beam_sides = []  # outward normal and offset of every side of the beam, in the same coordinates
    for s in beam.ref_sides:
        origin = Point(*side.to_local_coordinates(s.point))
        normal = Vector(*side.to_local_coordinates(s.point + s.normal)) - origin
        beam_sides.append((normal, normal.dot(origin)))

    # (a_u, a_v, a_w, b): inside where a_u*u + a_v*v <= b + a_w*depth, already offset for the bit
    constraints = []
    undercut = False
    for face in faces:
        a, b, c = (local[i] for i in face[:3])
        normal = Vector(*(b - a).cross(c - a)).unitized()
        if normal.dot(centre - a) > 0:
            normal = -normal
        offset = normal.dot(a)
        in_plane = math.hypot(normal[0], normal[1])
        is_open = any(normal.dot(n) > 1 - 1e-6 and offset >= o - 1e-3 for n, o in beam_sides)
        margin = -(radius + CLEARANCE) if is_open else radius
        undercut = undercut or (not is_open and normal[2] > 1e-6)
        constraints.append((normal[0], normal[1], normal[2], offset - margin * in_plane))

    def interval(u, depth):
        lo, hi = -math.inf, math.inf
        for a_u, a_v, a_w, b in constraints:
            for d in (0.0, depth):  # at this depth and at the surface
                rhs = b + a_w * d - a_u * u
                if abs(a_v) < 1e-9:
                    if rhs < -1e-9:
                        return None
                elif a_v > 0:
                    hi = min(hi, rhs / a_v)
                else:
                    lo = max(lo, rhs / a_v)
        return (lo, hi) if hi - lo > -1e-9 else None

    us = [p[0] for p in local]
    deepest = max(-p[2] for p in local)
    samples = [min(us) + i * 0.1 for i in range(int((max(us) - min(us)) / 0.1) + 1)]

    def frame(u, v, depth):
        point = side.point + side.xaxis * u + side.yaxis * v - side.normal * depth
        return Frame(point, side.xaxis, Vector(*(-side.normal)).cross(side.xaxis))

    frames = []
    layers = max(1, int(math.ceil(deepest / stepdown)))
    for layer in range(1, layers + 1):
        depth = deepest * layer / layers
        reachable = [u for u in samples if interval(u, depth)]
        if not reachable:
            continue  # nothing at this depth the bit fits in: the volume narrows below the bit
        first, last = reachable[0], reachable[-1]
        lines = max(1, int(math.ceil((last - first) / stepover)) + 1)
        layer_frames = []
        for line in range(lines):
            u = first + (last - first) * line / max(lines - 1, 1)
            span = interval(u, depth)
            if span is None:
                continue
            a, b = span if len(layer_frames) % 4 == 0 else span[::-1]
            layer_frames += [frame(u, a, depth), frame(u, b, depth)]
        if layer_frames:
            first_point, last_point = layer_frames[0].point, layer_frames[-1].point
            above = side.normal * (depth + safe_height)
            frames.append(Frame(first_point + above, layer_frames[0].xaxis, layer_frames[0].yaxis))
            frames += layer_frames
            frames.append(Frame(last_point + above, layer_frames[-1].xaxis, layer_frames[-1].yaxis))
    if not frames:
        return frames, "the bit can't get into it from this side"
    note = f"pocket, {len(frames)} frames"
    return frames, note + (", UNDERCUT: a wall leans over it, the bit leaves that wedge" if undercut else "")


def _pocket(corner, along, across, down, length, width, top, bottom, bit_diameter, stepdown, safe_height):
    """Zig-zag a closed rectangular pocket, `length` along `along` by `width` along `across` from `corner`,
    from `top` to `bottom` below the surface, the lines along its longer side. Each layer starts from `safe_height` above the surface."""
    radius = bit_diameter / 2.0
    if min(length, width) < bit_diameter - 1e-6:
        raise ValueError(f"A {bit_diameter:.0f} mm bit doesn't fit in a {length:.1f} x {width:.1f} mm pocket.")
    if width > length:
        along, across, length, width = across, along, width, length
    usable = width - bit_diameter
    lines = max(1, int(math.ceil(usable / (bit_diameter * 0.6))) + 1) if usable > 1e-6 else 1
    positions = [radius + usable * i / (lines - 1) for i in range(lines)] if lines > 1 else [width / 2.0]
    layers = max(1, int(math.ceil((bottom - top) / stepdown)))
    yaxis = Vector(*down.cross(along))

    def frame(at_along, at_across, at_depth):
        return Frame(corner + along * at_along + across * at_across + down * at_depth, along, yaxis)

    frames = []
    for layer in range(1, layers + 1):
        depth = top + (bottom - top) * layer / layers
        frames.append(frame(radius, positions[0], -safe_height))
        for line, at_across in enumerate(positions):
            a, b = (radius, length - radius) if line % 2 == 0 else (length - radius, radius)
            frames.append(frame(a, at_across, depth))
            frames.append(frame(b, at_across, depth))
        frames.append(frame(b, positions[-1], -safe_height))
    return frames


def _mortise_box(beam, mortise):
    """The corner, the length and width directions and the into-the-wood direction of a mortise's box, as compas_timber builds it."""
    if abs(mortise.slope - 90.0) > 1e-6 or abs(mortise.inclination - 90.0) > 1e-6:
        raise ValueError("Only mortises with square walls (slope and inclination 90°) are milled for now.")
    if mortise.shape not in ("square", "automatic"):
        raise ValueError(f"Only square mortises are milled for now (corners rounded to the bit), not {mortise.shape}.")
    start = mortise.frame_from_params_and_beam(beam)
    across = Vector(*start.xaxis)
    return Point(*start.point) - across * (mortise.width / 2.0), Vector(*(-start.yaxis)), across, Vector(*(-start.normal))


def mortise_toolpath(beam, mortise, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear a `Mortise` as a closed pocket. Its corners come out rounded to the bit's radius, whatever its `shape`.

    Like compas_timber's geometry, it ignores `start_depth`: the pocket is `depth` deep from the reference side.
    """
    corner, along, across, down = _mortise_box(beam, mortise)
    return _pocket(corner, along, across, down, mortise.length, mortise.width, 0.0, mortise.depth, bit_diameter, stepdown, safe_height)


def house_mortise_toolpath(beam, house, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear a `HouseMortise`, then the mortise nested in it, from the house's bottom (its `start_depth`) down."""
    frames = mortise_toolpath(beam, house, bit_diameter, stepdown, safe_height)
    for nested in house.subprocessings or []:
        if type(nested) is not Mortise:
            raise ValueError(f"A nested {type(nested).__name__} isn't milled with the {bit_diameter:.0f} mm bit.")
        corner, along, across, down = _mortise_box(beam, nested)
        frames += _pocket(corner, along, across, down, nested.length, nested.width, nested.start_depth, nested.start_depth + nested.depth, bit_diameter, stepdown, safe_height)
    return frames


def drilling_toolpath(beam, drilling, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0, segments=36):
    """Mill a `Drilling`: the bit spirals down the hole's wall, one turn per `stepdown`, and after each turn clears the core ring by ring.

    The hole is `depth` deep, measured square to the reference side, or through. A hole as wide as the bit is plunged.
    compas_timber draws the hole 10% longer than that at both ends; the toolpath follows the parameters.
    """
    radius = bit_diameter / 2.0
    if drilling.diameter < bit_diameter - 1e-6:
        raise ValueError(f"A {bit_diameter:.0f} mm bit doesn't fit in a {drilling.diameter:.1f} mm hole.")
    cylinder = drilling.cylinder_from_params_and_element(beam)
    axis = Vector(*cylinder.frame.zaxis)
    length = cylinder.height / 1.2  # undo compas_timber's 10% at either end
    top = Point(*cylinder.frame.point) - axis * (length / 2.0)
    u, v = Vector(*cylinder.frame.xaxis), Vector(*cylinder.frame.yaxis)
    ring = drilling.diameter / 2.0 - radius

    # a tilted hole's mouth is a slanted ellipse: start and end far enough out that the whole bit is in the air
    tilt = axis.angle(-beam.ref_sides[drilling.ref_side_index].normal)
    overhang = (ring + radius) * math.tan(tilt)
    start = -overhang - 1.0
    end = length if drilling.depth_limited else length + overhang + CLEARANCE

    def frame(r, angle, along):
        return Frame(top + u * (r * math.cos(angle)) + v * (r * math.sin(angle)) + axis * along, u, v)

    frames = [frame(0.0, 0.0, start - safe_height)]
    if ring < 1e-6:
        return frames + [frame(0.0, 0.0, end), frame(0.0, 0.0, start - safe_height)]
    frames.append(frame(ring, 0.0, start))
    turns = max(1, int(math.ceil((end - start) / stepdown)))
    for turn in range(turns):
        z0, z1 = start + (end - start) * turn / turns, start + (end - start) * (turn + 1) / turns
        frames += [frame(ring, 2 * math.pi * k / segments, z0 + (z1 - z0) * k / segments) for k in range(1, segments + 1)]
        frames += [frame(ring, 2 * math.pi * k / segments, z1) for k in range(1, segments + 1)]
        r = ring
        while r - radius > 1e-6:  # the core still stands inside the last ring
            r = max(r - bit_diameter * 0.6, 0.0)
            frames += [frame(r, 2 * math.pi * k / segments, z1) for k in range(segments + 1 if r > 1e-6 else 1)]
        if r < ring:
            frames.append(frame(ring, 0.0, z1))
    frames.append(frame(ring, 0.0, start - safe_height))
    return frames


def marking_lines(beam, marking):
    """The lines of a `Marking`, as (start, end) points on its reference side; a `square` is its four sides."""
    side = beam.ref_sides[marking.ref_side_index]
    side_width = beam.get_dimensions_relative_to_side(marking.ref_side_index)[0]
    angle = math.radians(marking.angle)
    first = (-math.cos(angle), math.sin(angle))  # from -X, clockwise looking at the side
    turn = math.radians(marking.interior_angle)  # clockwise from the first line to `width`
    second = (first[0] * math.cos(turn) + first[1] * math.sin(turn), -first[0] * math.sin(turn) + first[1] * math.cos(turn))
    p0 = (marking.start_x, marking.start_y)
    p1 = (p0[0] + second[0] * marking.width, p0[1] + second[1] * marking.width)

    def line(p):
        if marking.length_limited:
            return p, (p[0] + first[0] * marking.length, p[1] + first[1] * marking.length)
        # across the whole side: the line through `p` clipped to the side's rectangle
        low, high = -1e9, 1e9
        for at, d, size in ((p[0], first[0], beam.blank_length), (p[1], first[1], side_width)):
            if abs(d) < 1e-9:
                continue
            a, b = (-at) / d, (size - at) / d
            low, high = max(low, min(a, b)), min(high, max(a, b))
        return (p[0] + first[0] * low, p[1] + first[1] * low), (p[0] + first[0] * high, p[1] + first[1] * high)

    if marking.style == MarkingStyleType.SINGLE:
        lines = [line(p0)]
    elif marking.style == MarkingStyleType.DOUBLE:
        lines = [line(p0), line(p1)]
    else:
        if not marking.length_limited:
            raise ValueError("A square Marking needs a length.")
        a, b = line(p0)
        c, d = line(p1)
        lines = [(a, b), (b, d), (d, c), (c, a)]
    return [tuple(Point(*side.point) + Vector(*side.xaxis) * x + Vector(*side.yaxis) * y for x, y in pair) for pair in lines]


def marking_toolpath(beam, marking, depth=MARK_DEPTH, safe_height=20.0):
    """Engrave a `Marking`'s lines `depth` deep, in and out of the wood at each line's ends."""
    down = Vector(*(-beam.ref_sides[marking.ref_side_index].normal))
    frames = []
    for start, end in marking_lines(beam, marking):
        along = (end - start).unitized()
        yaxis = down.cross(along)
        frames += [Frame(point, along, yaxis) for point in (start - down * safe_height, start + down * depth, end + down * depth, end - down * safe_height)]
    return frames


def chamfer_toolpath(beam, chamfer, side_index=0, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Face a `Chamfer` with the bit square to it, on the chamfered edges of side `side_index` (the one facing up).

    Layers parallel to the chamfer, the last on it, each in lines along the edge. Through ends run out of the
    wood; a limited end stops where the bit's edge reaches it, so it comes out rounded to the bit's radius.
    """
    if ChamferExitType.ORTHOGONAL != chamfer.chamfer_exit and (chamfer.start_limited or chamfer.end_limited):
        raise ValueError(f"Only orthogonal chamfer exits are milled for now, not {chamfer.chamfer_exit}.")
    radius = bit_diameter / 2.0
    height = chamfer.depth / math.sqrt(2.0)  # from the edge to the chamfer, square to it
    x0 = chamfer.start_x + radius if chamfer.start_limited else -radius - CLEARANCE
    x1 = chamfer.start_x + chamfer.length - radius if chamfer.end_limited else beam.blank_length + radius + CLEARANCE
    if x1 <= x0:
        raise ValueError(f"A {bit_diameter:.0f} mm bit doesn't fit in a {chamfer.length:.1f} mm long chamfer.")
    frames = []
    for i, name in enumerate(CHAMFER_EDGES):
        if not getattr(chamfer, "chamfer_edge_" + name) or side_index not in (i, (i + 1) % 4):
            continue
        side, next_side = beam.ref_sides[i], beam.ref_sides[(i + 1) % 4]  # side i's reference edge is its edge to side i + 1
        out = Vector(*(side.normal + next_side.normal)).unitized()
        across = Vector(*(side.normal - next_side.normal)).unitized()
        xaxis = Vector(*side.xaxis)
        yaxis = (-out).cross(xaxis)
        layers = max(1, int(math.ceil(height / stepdown)))
        for layer in range(1, layers + 1):
            over = height * (layers - layer) / layers  # above the chamfer
            half = height - over  # half the layer's width, the wood there is a triangle
            lines = max(1, int(math.ceil(2 * (half - radius) / (bit_diameter * 0.6))) + 1) if half > radius else 1
            positions = [-(half - radius) + 2 * (half - radius) * k / (lines - 1) for k in range(lines)] if lines > 1 else [0.0]

            def frame(x, at_across, at_over):
                return Frame(Point(*side.point) + xaxis * x - out * (height - at_over) + across * at_across, xaxis, yaxis)

            frames.append(frame(x0, positions[0], over + safe_height))
            for line, at_across in enumerate(positions):
                a, b = (x0, x1) if line % 2 == 0 else (x1, x0)
                frames += [frame(a, at_across, over), frame(b, at_across, over)]
            frames.append(frame(b, positions[-1], over + safe_height))
    return frames


def _tenon_box(beam, tenon):
    """The tenon in its reference side's coordinates: the shoulder's x, the direction it points along x (+1 or -1),
    its y range, its depth range below the side, and the x where the timber ends on its side."""
    if any(abs(a - 90.0) > 1e-6 for a in (tenon.angle, tenon.inclination, tenon.rotation)):
        raise ValueError("Only square tenons (angle, inclination and rotation 90°) are milled for now.")
    if tenon.shape != "square" or tenon.chamfer:
        raise ValueError(f"Only square, unchamfered tenons are milled for now, not {tenon.shape}{' chamfered' if tenon.chamfer else ''}.")
    pointing = 1.0 if tenon.orientation == OrientationType.END else -1.0
    y0, y1 = tenon.start_y - tenon.width / 2.0, tenon.start_y + tenon.width / 2.0
    return tenon.start_x, pointing, (y0, y1), (tenon.start_depth, tenon.start_depth + tenon.length), beam.blank_length if pointing > 0 else 0.0


def tenon_toolpath(beam, tenon, from_bottom=False, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear around a `Tenon` from its reference side, or `from_bottom`, the beam flipped onto it.

    Each side clears its half of the beam, split at the beam's middle (or the tenon's top or bottom, if the
    middle misses it): the full width down to the tenon, then beside it, and past its tip to the end of the
    timber. Lines run along the beam, in from the open end and back out, so the bit only steps over in the air.
    The inner corners at the shoulder come out rounded to the bit's radius; the tenon's own edges stay sharp.

    Returns
    -------
    tuple(list[Frame], float)
        Bit tip positions, approach and retract included, and how deep the bit goes from that side.
    """
    radius = bit_diameter / 2.0
    stepover = bit_diameter * 0.6
    shoulder, pointing, (y0, y1), (top, bottom), end = _tenon_box(beam, tenon)
    width, height = beam.get_dimensions_relative_to_side(tenon.ref_side_index)
    split = min(max(height / 2.0, top), bottom)
    near, deepest = (height - bottom, height - split) if from_bottom else (top, split)  # from the side milled

    side = beam.ref_sides[tenon.ref_side_index]
    out = Vector(*side.xaxis) * pointing
    across = Vector(*side.yaxis)
    down = Vector(*(-side.normal))
    origin = Point(*side.point) + Vector(*side.xaxis) * shoulder
    if from_bottom:
        origin, down = origin + down * height, -down
    outside = (end - shoulder) * pointing + radius + CLEARANCE
    beyond_tip = (end - shoulder) * pointing > tenon.height + 1e-6

    def spread(a, b):
        """Positions from a to b, at most a stepover apart."""
        n = max(1, int(math.ceil((b - a) / stepover)))
        return [a + (b - a) * i / n for i in range(n + 1)] if b - a > 1e-6 else [a]

    def frame(along, at_across, at_depth):
        return Frame(origin + out * along + across * at_across + down * at_depth, out, down.cross(out))

    above = [(y, radius) for y in spread(0.0, width)]
    beside = [(y, radius) for y in spread(min(0.0, y0 - radius), y0 - radius) + spread(y1 + radius, max(width, y1 + radius))]
    past = [(y, tenon.height + radius) for y in spread(y0 - radius, y1 + radius)[1:-1]] if beyond_tip else []
    layers = [(near * k / n, above) for n in [int(math.ceil(near / stepdown))] for k in range(1, n + 1)]
    layers += [(near + (deepest - near) * k / n, sorted(beside + past)) for n in [int(math.ceil((deepest - near) / stepdown))] for k in range(1, n + 1)]

    frames = []
    for depth, lines in layers:
        frames.append(frame(outside, lines[0][0], -safe_height))
        for at_across, inside in lines:
            frames += [frame(outside, at_across, depth), frame(inside, at_across, depth), frame(outside, at_across, depth)]
        frames.append(frame(outside, lines[-1][0], -safe_height))
    return frames, deepest


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

    # Self-check: pockets, a mortise and drillings reach their walls, less the bit's radius, run past open sides, and say so when they can't.
    from compas_timber.fabrication import Drilling, Mortise, Pocket

    def cut_extents(processing):
        beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=120)
        beam.add_features(processing)
        (toolpath,) = toolpaths(beam)
        side = beam.ref_sides[processing.ref_side_index]
        cut = [side.to_local_coordinates(f.point) for f in toolpath.frames]
        cut = [p for p in cut if -p[2] > 1e-6]
        return [(round(min(p[i] for p in cut), 6), round(max(p[i] for p in cut), 6)) for i in range(3)], toolpath.note

    r = BIT_DIAMETER / 2
    extents, note = cut_extents(Pocket(start_x=300, start_y=20, start_depth=25, length=100, width=40))
    assert extents == [(300 + r, 400 - r), (20 + r, 60 - r), (-25, extents[2][1])] and "UNDERCUT" not in note, (extents, note)
    extents, _ = cut_extents(Pocket(start_x=300, start_y=0, start_depth=25, length=100, width=80))
    assert extents[1] == (-r - CLEARANCE, 80 + r + CLEARANCE), extents
    extents, note = cut_extents(Pocket(start_x=300, start_y=20, start_depth=25, length=100, width=40, tilt_start_side=60))
    assert "UNDERCUT" in note, note
    extents, _ = cut_extents(Mortise(start_x=500, start_y=40, length=60, width=20, depth=40))
    assert extents == [(500 + r, 560 - r), (30 + r, 50 - r), (-40, extents[2][1])], extents
    extents, _ = cut_extents(Drilling(start_x=800, start_y=40, depth=30, depth_limited=True, diameter=20))
    assert abs(extents[0][0] - (800 - 10 + r)) < 1e-6 and abs(extents[0][1] - (800 + 10 - r)) < 1e-6 and extents[2][0] == -30, extents
    extents, _ = cut_extents(Drilling(start_x=700, start_y=40, depth=30, depth_limited=True, diameter=6))
    assert extents[0] == (700, 700) and extents[2][0] == -30, extents
    print("Pockets, mortises and drillings reach their walls, run past open sides, and flag undercuts.")

    # Self-check: a mortise's pocket stays inside its box, a bit's radius from the walls, down to its depth.
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=120)
    mortise = Mortise(start_x=300.0, start_y=40.0, angle=30.0, length=60.0, width=20.0, depth=40.0, shape="square")
    corner, along, across, down = _mortise_box(beam, mortise)
    for f in mortise_toolpath(beam, mortise):
        a, c, d = ((f.point - corner).dot(axis) for axis in (along, across, down))
        if d > 1e-6:
            assert d <= mortise.depth + 1e-6 and 3 - 1e-6 <= a <= mortise.length - 3 + 1e-6 and 3 - 1e-6 <= c <= mortise.width - 3 + 1e-6, (a, c, d)
    print("Mortise pocket stays inside the mortise.")

    # Self-check: a drilled hole's bit stays inside the hole, and goes as deep as the hole.
    drilling = Drilling(start_x=500.0, start_y=40.0, inclination=70.0, depth_limited=True, depth=50.0, diameter=20.0)
    cylinder = drilling.cylinder_from_params_and_element(beam)
    frames = drilling_toolpath(beam, drilling)
    distances = [Vector(*(f.point - cylinder.frame.point)).cross(cylinder.frame.zaxis).length for f in frames]
    assert max(distances) <= drilling.diameter / 2 - 3 + 1e-6
    assert abs(max((f.point - cylinder.frame.point).dot(cylinder.frame.zaxis) for f in frames) - cylinder.height / 2.4) < 1e-6
    print(f"Drilling stays inside the hole ({len(frames)} frames).")

    # Self-check: a single marking line reads back the line it was made from.
    side = beam.ref_sides[1]
    line = Line(side.to_world_coordinates(Point(100, 20, 0)), side.to_world_coordinates(Point(160, 50, 0)))
    marking = Marking.from_line_and_element(line, beam, ref_side_index=1)
    ((start, end),) = marking_lines(beam, marking)
    assert start.distance_to_point(line.start) < 1e-6 and end.distance_to_point(line.end) < 1e-6, (start, end)
    print("Marking lines read back the line they came from.")

    # Self-check: chamfer facing never cuts below the chamfer, and its last layer is on it.
    chamfer = Chamfer(depth=10.0, chamfer_edge_12=True, chamfer_edge_23=False, chamfer_edge_34=False, chamfer_edge_41=False)
    side, next_side = beam.ref_sides[0], beam.ref_sides[1]
    out = Vector(*(side.normal + next_side.normal)).unitized()
    over = [(f.point - side.point).dot(out) + 10.0 / math.sqrt(2) for f in chamfer_toolpath(beam, chamfer)]
    assert min(over) > -1e-6 and abs(min(over)) < 1e-6
    assert chamfer_toolpath(beam, chamfer, side_index=2) == []
    print("Chamfer facing stays on the chamfer.")

    # Self-check: a tenon's two setups never touch the tenon, and together clear everything else past the shoulder.
    for tenon in (
        Tenon(orientation=OrientationType.END, start_x=950.0, start_y=40.0, start_depth=30.0, angle=90.0, inclination=90.0, rotation=90.0, length=60.0, width=20.0, height=40.0, shape="square"),
        Tenon(orientation=OrientationType.START, start_x=40.0, start_y=10.0, start_depth=0.0, angle=90.0, inclination=90.0, rotation=90.0, length=50.0, width=20.0, height=40.0, shape="square"),
    ):
        beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=120)
        beam.add_features(tenon)
        top, bottom = toolpaths(beam)
        assert (top.ref_side_index, bottom.ref_side_index) == (0, 2)
        shoulder, pointing, (y0, y1), (d0, d1), end = _tenon_box(beam, tenon)
        side = beam.ref_sides[0]

        def local(f):  # along from the shoulder, across, depth below side 0, and whether the bit hangs down from the tip
            x, y, z = side.to_local_coordinates(f.point)
            return (x - shoulder) * pointing, y, -z, Vector(*f.zaxis).dot(side.normal) > 0

        cuts = []  # (start, end, depth, from below) of every move at depth
        for t in (top, bottom):
            for a, b in zip(t.frames, t.frames[1:]):
                (sa, ya, da, up), (sb, yb, db, _) = local(a), local(b)
                if abs(da - db) < 1e-6:
                    cuts.append(((sa, ya), (sb, yb), da, up))

        def reaches(cut, s, y, d, margin):
            (sa, ya), (sb, yb), depth, up = cut
            if (d < depth - margin) if up else (d > depth + margin):
                return False
            ab, ap = (sb - sa, yb - ya), (s - sa, y - ya)
            k = max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1]) / ((ab[0] ** 2 + ab[1] ** 2) or 1.0)))
            return math.hypot(s - sa - k * ab[0], y - ya - k * ab[1]) <= BIT_DIAMETER / 2 + margin

        for s in [i * 2.0 for i in range(int(abs(end - shoulder) / 2) + 1)]:
            for y in [i * 2.0 for i in range(41)]:
                for d in [i * 2.0 for i in range(61)]:
                    in_tenon = s <= tenon.height + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6 and d0 - 1e-6 <= d <= d1 + 1e-6
                    if in_tenon:
                        assert not any(reaches(c, s, y, d, -1e-3) for c in cuts), ("cuts the tenon", s, y, d)
                    elif s > 3.0 and not (s < 3.0 + 1e-6 or (s < 6.0 and (abs(y - y0) < 3.0 or abs(y - y1) < 3.0))):
                        assert any(reaches(c, s, y, d, 1e-3) for c in cuts), ("left standing", s, y, d)
    print("Tenon setups clear around the tenon, and only around it.")
