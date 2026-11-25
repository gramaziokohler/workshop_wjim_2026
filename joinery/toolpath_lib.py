"""Toolpaths: from a beam's BTLx processings to frames for the milling bit.

Every frame is a position of the bit's tip, its Z axis pointing into the
material. What gets a toolpath today:

- `Lap`: a pocket, cleared in zig-zag lines, layer by layer.
- `StepJointNotch`: cleared with the bit square to its seat, the long face
  the strut sits on.
- `Pocket`, `Mortise`, `Slot`: any convex volume, cleared in zig-zag layers with the bit
  square to the ref side. Inside corners stay round, at the bit's radius.
- `HouseMortise`: a closed pocket, zig-zag along its long side, then the mortise
  nested in it; the corners come out rounded to the bit's radius.
- `Drilling`: a helix down the hole, the core cleared ring by ring.
- `Marking`, `Text`: lines engraved `MARK_DEPTH` deep; text in compas_timber's single-stroke font.
- `Chamfer`: faced with the bit square to the chamfer, layer by layer, on the
  edges of the side facing up.
- `Tenon`: as many setups as the bit's reach asks for, each its own `Toolpath`. If half the beam is within it, two:
  the top half from the reference side, the bottom half with the beam flipped onto it. If not, up to four: the face
  of the tenon towards each side is cleared from that side. What is out of reach is left standing, and the note says so.
  A `House` is one, and the tenon nested in it is cleared around after it.
- `JackRafterCut`, `DoubleCut`, `StepJoint`, `LongitudinalCut` (unlimited), `TriangleCut`, `RidgeValleyCut`,
  `HipValleyRafterNotch`: what they remove is cleared like a `Pocket`, from the ref side, or from the opposite
  side if a face leans over it from the ref side only. Some remove two volumes, each its own `Toolpath`.
- `BirdsMouth`: the V cleared like a `Pocket`, its sloped faces left in steps of a layer.
- `LogHouseHalfLap`: two through laps, from the reference side and the beam flipped. Two `Toolpath`s.
- `Planing`: each planed side faced, in a setup of its own.
- `FreeContour`, `MillContour`: followed with the bit on its `tool_position` side, layer by layer;
  a closed, convex, not-through contour whose inside is removed is cleared as a pocket.

A `JackRafterCut` can be sawn by hand instead: `saw_guides` swaps its toolpath for two grooves to saw along.

Anything else is listed as "no toolpath yet" - that list is the honest answer to "can we mill this joint?".

Adapted from the CAADRIA 2026 workshop's `lib_extra_toolpath_v2.py`.
Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import itertools
import math

from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import intersection_plane_plane_plane
from compas.geometry import offset_polygon
from compas.geometry import offset_polyline
from compas_timber.fabrication import AlignmentType
from compas_timber.fabrication import BirdsMouth
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import Drilling
from compas_timber.fabrication import DualContour
from compas_timber.fabrication import FreeContour
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap
from compas_timber.fabrication import LongitudinalCut
from compas_timber.fabrication import Mortise
from compas_timber.fabrication import OrientationType
from compas_timber.fabrication import Pocket
from compas_timber.fabrication import Slot
from compas_timber.fabrication import StepJoint
from compas_timber.fabrication import StepJointNotch
from compas_timber.fabrication import StepShapeType
from compas_timber.fabrication import Tenon
from compas_timber.fabrication import Text
from wjim_lib import Chamfer
from wjim_lib import ChamferExitType
from wjim_lib import House
from wjim_lib import HipValleyRafterNotch
from wjim_lib import HouseMortise
from wjim_lib import LogHouseHalfLap
from wjim_lib import Marking
from wjim_lib import MarkingStyleType
from wjim_lib import Planing
from wjim_lib import RidgeValleyCut
from wjim_lib import SawContour
from wjim_lib import TriangleCut

BIT_DIAMETER = 6.0  # mm, the bit on site
CLEARANCE = 5.0  # mm between the bit and a beam edge it cuts through, so it goes in and out of the wood in the air
# What hangs above the bit: (mm above its tip, radius in mm) where each step of it starts, as in robot/milling_tool.json.
# How far the bit sticks out decides how deep it goes next to wood that stays: measure it, and change this with the bit.
SPINDLE = ((35.0, 25.0), (81.0, 45.0), (121.0, 88.0))
SPINDLE_CLEARANCE = 1.0  # mm kept between the spindle and wood that stays
MARK_DEPTH = 1.0  # mm, how deep a Marking is engraved
GUIDE_DEPTH = 2.0  # mm, how deep a saw guide is engraved
MILLED_CUTS = (JackRafterCut, DoubleCut, StepJoint, LongitudinalCut, TriangleCut, RidgeValleyCut, HipValleyRafterNotch)
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
        elif isinstance(processing, (Pocket, Mortise, Slot)):
            frames, note = pocket_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
        elif isinstance(processing, Drilling):
            frames = drilling_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
            note = f"helix, {len(frames)} frames"
        elif isinstance(processing, (Marking, Text)):
            frames = (marking_toolpath if isinstance(processing, Marking) else text_toolpath)(beam, processing, safe_height=safe_height)
            note = f"engraved {MARK_DEPTH:.0f} mm deep with the {bit_diameter:.0f} mm bit, {len(frames)} frames"
        elif isinstance(processing, Chamfer):
            frames = chamfer_toolpath(beam, processing, side, bit_diameter, stepdown, safe_height)
            elsewhere = [name for i, name in enumerate(CHAMFER_EDGES) if getattr(processing, "chamfer_edge_" + name) and side not in (i, (i + 1) % 4)]
            note = f"chamfer facing, {len(frames)} frames" + (f"; edges {', '.join(elsewhere)} need the beam turned" if elsewhere else "")
        elif isinstance(processing, Tenon):  # a House is one
            for at, frames, note in tenon_setups(beam, processing, bit_diameter, stepdown, safe_height):
                result.append(Toolpath(processing, frames, at, note))
            continue
        elif isinstance(processing, MILLED_CUTS):
            for planes in cut_volumes(beam, processing):
                # a face leaning over the volume is an undercut: mill from the opposite side if none leans over it from there
                leans = [any(Vector(*p.normal).dot(beam.ref_sides[s].normal) > 1e-6 for p in planes) for s in (side, (side + 2) % 4)]
                at = (side + 2) % 4 if leans == [True, False] else side
                frames, note = convex_toolpath(beam, planes + _beam_planes(beam), at, bit_diameter, stepdown, safe_height, name=f"cleared from side {at}")
                result.append(Toolpath(processing, frames, at, note))
            continue
        elif isinstance(processing, BirdsMouth):
            frames, note = convex_toolpath(beam, birds_mouth_planes(beam, processing), side, bit_diameter, stepdown, safe_height, name="birds mouth")
        elif isinstance(processing, (LogHouseHalfLap, Planing)):
            volumes = log_half_lap_volumes(beam, processing) if isinstance(processing, LogHouseHalfLap) else planing_volumes(beam, processing)
            for at, planes in volumes:
                frames, note = convex_toolpath(beam, planes, at, bit_diameter, stepdown, safe_height, name=f"{'lap' if isinstance(processing, LogHouseHalfLap) else 'planing'} from side {at}")
                result.append(Toolpath(processing, frames, at, note))
            continue
        elif isinstance(processing, FreeContour) and not isinstance(processing, SawContour):  # MillContour is one
            frames, note = contour_toolpath(beam, processing, bit_diameter, stepdown, safe_height)
        else:
            frames, note = [], "no toolpath for this processing yet"
        result.append(Toolpath(processing, frames, side, note))
    return result


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


def _face_planes(vertices, faces):
    """The planes of a convex polyhedron's faces, their normals pointing out of it."""
    centre = Point(*[sum(c) / len(vertices) for c in zip(*vertices)])
    planes = []
    for face in faces:
        a, b, c = (Point(*vertices[i]) for i in face[:3])
        normal = Vector(*(b - a).cross(c - a)).unitized()
        planes.append(Plane(a, -normal if normal.dot(centre - a) > 0 else normal))
    return planes


def _beam_planes(beam):
    """The six sides of the beam as planes, normals out of it: add them to bound a volume by the beam."""
    return [Plane(side.point, side.normal) for side in beam.ref_sides]


def _corners(planes):
    """The corners of the convex volume inside all `planes` (normals pointing out of it)."""
    corners = []
    for p, q, r in itertools.combinations(planes, 3):
        if abs(Vector(*p.normal).cross(q.normal).dot(r.normal)) < 1e-9:
            continue  # two of them are parallel: no single corner
        x = intersection_plane_plane_plane(p, q, r)
        if all((Point(*x) - s.point).dot(s.normal) < 1e-4 for s in planes):
            corners.append(Point(*x))
    return corners


def _volume(beam, processing):
    """The convex volume a `Pocket`, `Slot` or `Mortise` removes, as planes, normals out of it."""
    if isinstance(processing, (Pocket, Slot)):
        volume = processing.volume_from_params_and_element(beam) if isinstance(processing, Pocket) else processing.volume_from_params_and_beam(beam)
        return _face_planes(volume.vertices, volume.faces)
    # Mortise: the box its own volume_from_params_and_beam builds, without going through a Brep.
    frame = processing.frame_from_params_and_beam(beam)
    frame.translate((-frame.normal * processing.depth - frame.yaxis * processing.length) * 0.5)
    return _face_planes(*Box(processing.width, processing.length, processing.depth, frame).to_vertices_and_faces())


def pocket_toolpath(beam, processing, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear the convex volume of a `Pocket`, `Slot` or `Mortise` with the bit square to its ref side, see `convex_toolpath`."""
    return convex_toolpath(beam, _volume(beam, processing), processing.ref_side_index, bit_diameter, stepdown, safe_height, name=type(processing).__name__.lower())


def convex_toolpath(beam, planes, side_index, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0, name="pocket"):
    """Clear the convex volume inside `planes` (normals pointing out of it) with the bit square to side `side_index`, in zig-zag layers.

    Every plane is a half-space. At each depth, the bit's centre may go where its whole
    disc is inside all of them, there and at the surface (so its shaft doesn't cut an overhang on the
    way down). A face flush with or outside a side of the beam is open: the bit runs past it by `CLEARANCE`.
    The lines run straight across the beam, or parallel to the walls where those are at an angle to it.

    Returns
    -------
    tuple
        Bit tip positions, approach and retract included (empty if the bit can't get in from this side),
        and the note, starting with `name`: a face that hangs over the volume is an undercut the bit leaves
        standing, a sloped one comes out in steps of a layer.
    """
    radius = bit_diameter / 2.0
    stepover = bit_diameter * 0.6
    ref_side = beam.ref_sides[side_index]
    is_open = [any(p.normal.dot(s.normal) > 1 - 1e-6 and (p.point - s.point).dot(s.normal) >= -1e-3 for s in beam.ref_sides) for p in planes]
    # the closed walls' normals on the side, (x, y): the lines run parallel to the one that turns them the least from straight across
    walls = [(p.normal.dot(ref_side.xaxis), p.normal.dot(ref_side.yaxis)) for p, o in zip(planes, is_open) if not o]
    walls = [(x / math.hypot(x, y), y / math.hypot(x, y)) for x, y in walls if math.hypot(x, y) > 1e-6]
    turn_x, turn_y = max(walls, key=lambda w: abs(w[0]), default=(1.0, 0.0))
    if abs(turn_x) < 1e-3:  # only walls along the beam: straight across is square to them
        turn_x, turn_y = 1.0, 0.0
    if turn_x < 0:
        turn_x, turn_y = -turn_x, -turn_y
    side = Frame(ref_side.point, ref_side.xaxis * turn_x + ref_side.yaxis * turn_y, ref_side.yaxis * turn_x - ref_side.xaxis * turn_y)
    local = [Point(*side.to_local_coordinates(c)) for c in _corners(planes)]  # z out of the side, depth = -z
    if not local:
        return [], f"{name}: nothing to mill, the volume is empty"

    # (a_u, a_v, a_w, b): inside where a_u*u + a_v*v <= b + a_w*depth, already offset for the bit
    constraints = []
    undercut = sloped = False
    for plane, opened in zip(planes, is_open):
        a = Point(*side.to_local_coordinates(plane.point))
        normal = (Vector(*side.to_local_coordinates(plane.point + plane.normal)) - a).unitized()
        offset = normal.dot(a)
        in_plane = math.hypot(normal[0], normal[1])
        margin = -(radius + CLEARANCE) if opened else radius
        undercut = undercut or (not opened and normal[2] > 1e-6)
        sloped = sloped or (not opened and 1e-6 < abs(normal[2]) < 1 - 1e-6)
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
        return Frame(point, ref_side.xaxis, Vector(*(-side.normal)).cross(ref_side.xaxis))  # the lines turn, the tool doesn't

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
        return frames, f"{name}: the bit can't get into it from this side"
    note = f"{name}, {len(frames)} frames" + (f", sloped faces come out in {deepest / layers:.1f} mm steps" if sloped else "")
    return frames, note + (", UNDERCUT: a wall leans over it, the bit leaves that wedge" if undercut else "")


def cut_volumes(beam, cut):
    """The convex volumes one of `MILLED_CUTS` removes, each as its planes, normals out of it, into the part that
    stays. Bound them by the beam's sides to get the volume."""
    if isinstance(cut, DoubleCut):
        planes = cut.planes_from_params_and_beam(beam)
        return [planes] if cut.is_concave else [[p] for p in planes]  # concave: one notch between both; convex: each cuts off its own corner
    if isinstance(cut, StepJoint):
        if cut.tenon:
            raise ValueError("A StepJoint's tenon isn't milled yet.")
        planes = cut.planes_from_params_and_beam(beam)  # as StepJoint.apply trims with them: each removes what is behind it
        return {StepShapeType.HEEL: [planes], StepShapeType.DOUBLE: [planes[2:], planes[:2]]}.get(cut.step_shape, [[p] for p in planes])
    if isinstance(cut, TriangleCut):
        start = (cut.start_x, cut.start_y, -cut.start_depth)
        normals = [(cut.normal_1_x, cut.normal_1_y, cut.normal_1_z), (cut.normal_2_x, cut.normal_2_y, cut.normal_2_z)]
        return [[_side_plane(beam, cut, start, [-c for c in n]) for n in normals]]  # its normals point into the wedge
    if isinstance(cut, RidgeValleyCut):
        return ridge_valley_volumes(beam, cut)
    if isinstance(cut, HipValleyRafterNotch):
        return hip_valley_notch_volumes(beam, cut)
    if isinstance(cut, LongitudinalCut) and (cut.start_limited or cut.end_limited or cut.depth_limited):
        raise ValueError("Only unlimited longitudinal cuts are milled for now (compas_timber can't model the limited ones either).")
    plane = cut.plane_from_params_and_beam(beam)
    return [[Plane(plane.point, -Vector(*plane.normal))]]  # the normal points into the offcut


def saw_guides(beam, toolpaths, bit_diameter=BIT_DIAMETER, depth=GUIDE_DEPTH, safe_height=20.0):
    """`toolpaths`, with each `JackRafterCut`'s swapped for two grooves to saw along, each its own `Toolpath`: on its ref
    side, at its angle, and on the side next to it that is squarest to the cut, at its inclination."""
    result = []
    for toolpath in toolpaths:
        cut = toolpath.processing
        if not isinstance(cut, JackRafterCut):
            result.append(toolpath)
            continue
        normal = Vector(*cut.plane_from_params_and_beam(beam).normal)
        ref = cut.ref_side_index or 0
        beside = min(sorted(((ref + 1) % 4, (ref + 3) % 4)), key=lambda i: round(abs(normal.dot(beam.ref_sides[i].normal)), 6))
        for at, name in ((ref, "angle"), (beside, "inclination")):
            frames, note = _saw_guide(beam, cut, at, bit_diameter, depth, safe_height)
            result.append(Toolpath(cut, frames, at, f"saw guide from side {at} ({name}), {note}"))
    return result


def _saw_guide(beam, cut, side_index, bit_diameter, depth, safe_height):
    """A groove in the offcut along `cut`'s line on a side, in and out of the wood in the air: its wall is the cut, the bit
    never goes into the part that stays. Where the cut leans over the groove, only the groove's bottom edge is on it."""
    side = beam.ref_sides[side_index]
    plane = cut.plane_from_params_and_beam(beam)  # its normal points into the offcut
    normal = Vector(*plane.normal).unitized()
    nx, ny, nz = (normal.dot(axis) for axis in (side.xaxis, side.yaxis, side.normal))
    across = math.hypot(nx, ny)
    if across < 1e-6:
        return [], "the cut doesn't cross this side"
    radius = bit_diameter / 2.0
    beside = max(nz, 0.0) * depth / across  # how far the groove's wall is from the cut's line on the side
    reach = normal.dot(plane.point - side.point) + (radius + beside) * across  # nx * x + ny * y = reach: the bit's line
    margin = radius + CLEARANCE
    (x0, x1), (y0, y1) = (-margin, beam.blank_length + margin), (-margin, beam.get_dimensions_relative_to_side(side_index)[0] + margin)
    ends = [(x, (reach - nx * x) / ny) for x in (x0, x1) if abs(ny) > 1e-9] + [((reach - ny * y) / nx, y) for y in (y0, y1) if abs(nx) > 1e-9]
    ends = sorted(e for e in ends if x0 - 1e-6 <= e[0] <= x1 + 1e-6 and y0 - 1e-6 <= e[1] <= y1 + 1e-6)
    if not ends or math.dist(ends[0], ends[-1]) < 1e-6:
        return [], "the cut doesn't cross this side"
    frames = _engrave(side, [[side.point + side.xaxis * x + side.yaxis * y for x, y in (ends[0], ends[-1])]], depth, safe_height)
    note = f"{depth:.0f} mm deep in the offcut, {len(frames)} frames"
    return frames, note + (f", its wall is {beside:.1f} mm past the cut's line" if beside > 0.05 else ", its wall is the cut's line")


def _side_plane(beam, processing, point, normal):
    """A plane given in the coordinates of `processing`'s ref side, in the beam's."""
    side = beam.ref_sides[processing.ref_side_index]
    return Plane(side.to_world_coordinates(Point(*point)), side.xaxis * normal[0] + side.yaxis * normal[1] + side.zaxis * normal[2])


def ridge_valley_volumes(beam, cut):
    """What a `RidgeValleyCut` removes, as planes: above each inclined plane, as `RidgeValleyCut.volumes_from_params_and_element`
    has it. Over a ridge that is everything above either plane, two volumes; in a valley, what is above both, one."""
    start = (cut.start_x, cut.start_y, -cut.start_depth)
    end = (cut.start_x + cut.length, cut.start_y, -cut.start_depth)
    halves = []
    for s, inclination, angle_start, angle_end in (
        (-1.0, cut.inclination_ref_side, cut.angle_ref_edge_start, cut.angle_ref_edge_end),
        (1.0, cut.inclination_opp_side, cut.angle_opp_edge_start, cut.angle_opp_edge_end),
    ):
        i, a0, a1 = math.radians(inclination), math.radians(angle_start), math.radians(angle_end)
        slope = Vector(0.0, s * math.cos(i), -math.sin(i))  # in the inclined plane, away from the ridge line
        planes = [_side_plane(beam, cut, start, (0.0, -s * math.sin(i), -math.cos(i)))]  # the inclined plane, normal down
        if cut.start_limited:
            planes.append(_side_plane(beam, cut, start, slope * math.cos(a0) - Vector(math.sin(a0), 0.0, 0.0)))
        if cut.end_limited:
            planes.append(_side_plane(beam, cut, end, slope * math.cos(a1) + Vector(math.sin(a1), 0.0, 0.0)))
        halves.append(planes)
    # shortcut: each side's start and end walls bound its whole volume, past the ridge line too; exact while both sides' walls agree
    return halves if cut.inclination_ref_side + cut.inclination_opp_side >= 0 else [halves[0] + halves[1]]


def hip_valley_notch_volumes(beam, notch):
    """What a `HipValleyRafterNotch` removes, as planes: above its floor, inside the chevron, as two volumes, one per arm.

    Each arm is a strip between its outer wall (through the tip) and its inner wall. Past the other arm, a hip's strip
    stops at that arm's outer wall, a valley's (arms falling away from the tip) at its inner wall."""
    side = beam.ref_sides[notch.ref_side_index]
    inclination = math.radians(notch.inclination)
    rise = side.xaxis * math.cos(inclination) * (1.0 if notch.orientation == OrientationType.START else -1.0) + side.zaxis * math.sin(inclination)
    tip = side.to_world_coordinates(Point(notch.start_x, notch.start_y, -notch.start_depth))
    floor = Frame(tip, rise, side.yaxis)
    up = floor.zaxis if floor.zaxis.dot(side.zaxis) >= 0 else -floor.zaxis

    def wall(normal, offset=0.0):  # a wall square to the floor, `normal` in the floor's xy, out of the volume
        n = floor.xaxis * normal[0] + floor.yaxis * normal[1]
        return Plane(tip + n * offset, n)

    a, b = math.radians(notch.angle_ref_edge), math.radians(notch.angle_opp_edge)
    arms = [  # per arm: its normal into it from its outer wall, its direction away from the tip, its width or None
        ((math.sin(a), math.cos(a)), (math.cos(a) / math.sin(a), -1.0), notch.width_counter_part_ref_edge if notch.width_counter_part_ref_edge_limited else None),
        ((math.sin(b), -math.cos(b)), (math.cos(b) / math.sin(b), 1.0), notch.width_counter_part_opp_edge if notch.width_counter_part_opp_edge_limited else None),
    ]
    volumes = []
    for (normal, _, width), (other_normal, direction, other_width) in (arms, arms[::-1]):
        planes = [Plane(tip, -up), wall([-c for c in normal])]
        if width is not None:
            planes.append(wall(normal, width))
        if direction[0] * normal[0] + direction[1] * normal[1] > 0:  # the other arm runs into this one's strip: a hip
            planes.append(wall([-c for c in other_normal]))
            if width is None:
                return [planes]  # the whole V between the outer walls, the other arm included
        elif other_width is not None:
            planes.append(wall(other_normal, other_width))
        volumes.append(planes)
    return volumes


def birds_mouth_planes(beam, notch):
    """The V a `BirdsMouth` removes: its two faces, normals out of the V, and the beam's sides."""
    side = beam.ref_sides[notch.ref_side_index]
    ridge = notch._get_ridge_line_from_params_and_element(beam)
    faces = notch.planes_from_params_and_element(beam)
    ups = []  # each face's direction from the ridge up to the surface
    for face in faces:
        up = Vector(*ridge.direction.cross(face.normal)).unitized()
        ups.append(up if up.dot(side.normal) > 0 else -up)
    inside = Point(*ridge.midpoint) + ups[0] + ups[1]  # between the faces, in the V
    planes = [Plane(f.point, -Vector(*f.normal) if (inside - f.point).dot(f.normal) > 0 else Vector(*f.normal)) for f in faces]
    return planes + _beam_planes(beam)


def log_half_lap_volumes(beam, lap):
    """The two notches of a `LogHouseHalfLap`, each as (the side it is milled from, its planes)."""
    side = beam.ref_sides[lap.ref_side_index]
    height = beam.get_dimensions_relative_to_side(lap.ref_side_index)[1]
    angle = math.radians(lap.angle)
    shift = 1.0 / math.tan(angle) * (-1.0 if lap.orientation == OrientationType.START else 1.0)  # the walls' change in x per unit of y
    half = 0.5 * lap.length / math.sin(angle)  # along the reference edge
    wall = Vector(*(side.xaxis - side.yaxis * shift)).unitized()  # square to the walls, towards the end
    walls = [Plane(side.point + side.xaxis * (lap.start_x + half), wall), Plane(side.point + side.xaxis * (lap.start_x - half), -wall)]
    down = Vector(*(-side.normal))
    volumes = []
    if lap.length and lap.depth_ref_side:
        volumes.append((lap.ref_side_index, walls + [Plane(side.point + down * lap.depth_ref_side, down)] + _beam_planes(beam)))
    if lap.length and lap.depth_opp_side:
        volumes.append(((lap.ref_side_index + 2) % 4, walls + [Plane(side.point + down * (height - lap.depth_opp_side), -down)] + _beam_planes(beam)))
    return volumes


def planing_volumes(beam, planing):
    """The layers a `Planing` takes off, each as (the side it is milled from, its planes)."""
    volumes = []
    for index in range(4):
        if not getattr(planing, f"plane_side_{index + 1}"):
            continue
        side = beam.ref_sides[index]
        planes = _beam_planes(beam) + [Plane(side.point - side.normal * planing.depth, -side.normal)]
        if planing.start_limited:
            planes.append(Plane(side.point + side.xaxis * planing.start_x, -side.xaxis))
        if planing.end_limited:
            planes.append(Plane(side.point + side.xaxis * (planing.start_x + planing.length), side.xaxis))
        volumes.append((index, planes))
    return volumes


def contour_toolpath(beam, contour, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Mill a `FreeContour` or a `MillContour` with the bit square to its ref side.

    A closed contour whose inside is removed (`counter_sink`), not through and convex, is cleared as a
    pocket. Any other contour is followed layer by layer, the bit on its `tool_position` side of it; an
    open contour's ends on the edge of the side run out of the wood.

    Returns
    -------
    tuple
        Bit tip positions, approach and retract included, and the note: what the bit leaves standing.
    """
    shape = contour.contour_param_object
    if isinstance(shape, DualContour) or any(abs(i) > 1e-6 for i in shape.inclination):
        raise ValueError("Only contours with square walls (inclination 0) are milled for now.")
    radius = bit_diameter / 2.0
    side_index = contour.ref_side_index
    side = beam.ref_sides[side_index]
    width, height = beam.get_dimensions_relative_to_side(side_index)
    # shortcut: the contour is taken to lie on the side (z = 0), as from_polyline_and_element makes it
    xy = [[p[0], p[1], 0.0] for p in shape.polyline]
    closed = shape.polyline.is_closed
    deepest = shape.depth if contour.depth_bounded else height + CLEARANCE
    down = Vector(*(-side.normal))

    def world(p, depth=0.0):
        return Point(*side.point) + Vector(*side.xaxis) * p[0] + Vector(*side.yaxis) * p[1] + down * depth

    ring = [Point(*p) for p in (xy[:-1] if closed else xy)]
    n = len(ring)
    turns = [(ring[i] - ring[i - 1]).cross(ring[(i + 1) % n] - ring[i])[2] for i in range(n)]  # + where it turns left
    if closed and contour.counter_sink and contour.depth_bounded and (min(turns) >= -1e-9 or max(turns) <= 1e-9):
        sign = 1.0 if sum(turns) > 0 else -1.0  # counterclockwise: the outside is right of every edge
        planes = []
        for a, b in zip(ring, ring[1:] + ring[:1]):
            out = (Vector(*side.xaxis) * (b[1] - a[1]) - Vector(*side.yaxis) * (b[0] - a[0])).unitized() * sign
            planes.append(Plane(world(a), out))
        planes.append(Plane(world([0, 0, 0], deepest), down))
        return convex_toolpath(beam, planes + _beam_planes(beam), side_index, bit_diameter, stepdown, safe_height, name="contour pocket")

    offset = {AlignmentType.LEFT: radius, AlignmentType.RIGHT: -radius}.get(contour.tool_position, 0.0)  # left is +, seen from outside the side
    if closed:
        path = offset_polygon(xy[:-1], offset) if offset else xy[:-1]
        path = path + path[:1]
    else:
        path = offset_polyline(xy, offset) if offset else xy
        for end, before in ((0, 1), (-1, -2)):  # an end on the side's edge runs out of the wood
            x, y = xy[end][0], xy[end][1]
            if min(x, beam.blank_length - x, y, width - y) < 1e-3:
                out = (Vector(*path[end]) - Vector(*path[before])).unitized() * (radius + CLEARANCE)
                path[end] = [path[end][0] + out[0], path[end][1] + out[1], 0.0]

    def frame(p, depth):
        return Frame(world(p, depth), side.xaxis, down.cross(side.xaxis))

    frames = [frame(path[0], -safe_height)]
    layers = max(1, int(math.ceil(deepest / stepdown)))
    for layer in range(1, layers + 1):
        run = path if closed or layer % 2 else path[::-1]  # an open contour goes back and forth
        frames += [frame(p, deepest * layer / layers) for p in run]
    frames.append(frame(run[-1], -safe_height))

    note = f"contour, {len(frames)} frames"
    if closed and not contour.depth_bounded:
        note += ", through: the " + ("inside" if contour.counter_sink else "outside") + " comes loose"
    elif closed:
        note += ", only along it: the " + ("inside isn't cleared, it isn't convex" if contour.counter_sink else "rest of the outside stays")
    return frames, note


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
    return _engrave(beam.ref_sides[marking.ref_side_index], marking_lines(beam, marking), depth, safe_height)


def text_toolpath(beam, text, depth=MARK_DEPTH, safe_height=20.0):
    """Engrave a `Text` `depth` deep, along the strokes of compas_timber's single-stroke font. The strokes are as
    wide as the bit, so letters much less than ten bits tall run together."""
    curves = text.create_text_curves_for_element(beam)
    return _engrave(beam.ref_sides[text.ref_side_index or 0], [curve.points for curve in curves], depth, safe_height)


def _engrave(side, polylines, depth, safe_height):
    """Follow each polyline on `side` `depth` deep, in and out of the wood at its ends."""
    down = Vector(*(-side.normal))
    frames = []
    for points in polylines:
        points = [Point(*p) for p in points]
        path = [points[0] - down * safe_height] + [p + down * depth for p in points] + [points[-1] - down * safe_height]
        frames += [Frame(p, side.xaxis, down.cross(side.xaxis)) for p in path]
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


def house_toolpath(beam, house, from_bottom=False, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """Clear around a `House`, then around the tenon nested in it, which stands on the house's end face, see `tenon_toolpath`."""
    nested = house.subprocessings or []
    if any(type(t) is not Tenon for t in nested):
        raise ValueError(f"Only a plain nested Tenon is milled with the {bit_diameter:.0f} mm bit.")
    frames, deepest = tenon_toolpath(beam, house, from_bottom, bit_diameter, stepdown, safe_height, past_tip=not nested)
    for tenon in nested:
        more, depth = tenon_toolpath(beam, tenon, from_bottom, bit_diameter, stepdown, safe_height)
        frames, deepest = frames + more, max(deepest, depth)
    return frames, deepest


def tenon_toolpath(beam, tenon, from_bottom=False, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0, past_tip=True):
    """Clear around a `Tenon` from its reference side, or `from_bottom`, the beam flipped onto it.

    Each side clears its half of the beam, split at the beam's middle (or the tenon's top or bottom, if the
    middle misses it): the full width down to the tenon, then beside it, and past its tip to the end of the
    timber. Lines run along the beam, in from the open end and back out, so the bit only steps over in the air.
    The inner corners at the shoulder come out rounded to the bit's radius; the tenon's own edges stay sharp.
    Without `past_tip`, the wood past the tip is left for what stands there (a `House`'s nested tenon).

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
    beyond_tip = past_tip and (end - shoulder) * pointing > tenon.height + 1e-6

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


def tenon_setups(beam, tenon, bit_diameter=BIT_DIAMETER, stepdown=3.0, safe_height=20.0):
    """The setups that mill a `Tenon` or `House` without the spindle touching wood that stays.

    Two, see `tenon_toolpath`, if the bit's reach allows it. Otherwise one per side with wood to clear around the tenon: the
    full width of the beam, from the shoulder to its end, down to the tenon's face towards that side. The wood past the tip
    goes from the two sides between which the tenon is thinnest.

    Returns
    -------
    list[tuple]
        The side milled from, the frames and the note of each setup. The note says how much is LEFT STANDING where the
        spindle would hit wood that stays before the bit is at its depth.
    """
    name, make = ("house", house_toolpath) if isinstance(tenon, House) else ("tenon", tenon_toolpath)
    ref = tenon.ref_side_index
    halves = [make(beam, tenon, from_bottom, bit_diameter, stepdown, safe_height) for from_bottom in (False, True)]
    if max(depth for _, depth in halves) <= SPINDLE[0][0] - SPINDLE_CLEARANCE:
        sides = ((ref, f"from side {ref}"), ((ref + 2) % 4, f"from side {(ref + 2) % 4}, the beam flipped"))
        return [(at, frames, f"{name} {how}, {depth:.0f} mm deep, {len(frames)} frames") for (at, how), (frames, depth) in zip(sides, halves)]

    radius = bit_diameter / 2.0
    solids = [tenon] + list((tenon.subprocessings or []) if isinstance(tenon, House) else [])
    boxes = [_tenon_box(beam, t) for t in solids]
    side = beam.ref_sides[ref]
    out = Vector(*side.xaxis) * boxes[0][1]
    origin = Point(*side.point) + Vector(*side.xaxis) * boxes[0][0]
    starts = [(shoulder - boxes[0][0]) * pointing for shoulder, pointing, _, _, _ in boxes]  # along the beam, from the first shoulder
    tips = [start + t.height for start, t in zip(starts, solids)]
    corners = [
        [side.to_world_coordinates(Point(box[0], y, -d)) for y in box[2] for d in box[3]] for box in boxes
    ]
    past_tip = (boxes[-1][4] - boxes[0][0]) * boxes[0][1] > tips[-1] + 1e-6

    def depths(at):  # of each solid's face towards side `at`, below it
        return [min(-beam.ref_sides[at].to_local_coordinates(c)[2] for c in points) for points in corners]

    def thickness(at):  # of the last solid, seen from side `at`
        a, b = beam.ref_sides[at], beam.ref_sides[(at + 2) % 4]
        return Vector(*(a.point - b.point)).dot(a.normal) - depths(at)[-1] - depths((at + 2) % 4)[-1]

    thin = min((ref, (ref + 1) % 4), key=thickness)
    setups = []
    for at in [(ref + k) % 4 for k in range(4)]:
        facing = beam.ref_sides[at]
        cuts = list(zip(starts, tips, depths(at)))
        if past_tip and at % 2 == thin % 2:
            cuts.append((tips[-1], math.inf, cuts[-1][2] + thickness(at) / 2.0))
        stays = [(0.0, 0.0)]  # (how far along the beam, how deep below this side) the wood that stays goes, seen from here
        frames, deepest, left = [], 0.0, 0.0
        for start, end, depth in cuts:
            limit = min(top + above - SPINDLE_CLEARANCE for reach, top in stays for above, around in SPINDLE if start + radius - reach < around)
            floor = min(depth, limit)
            left = max(left, depth - floor)
            stays.append((end if floor == depth else math.inf, floor))
            if floor < 1e-6:
                continue  # flush with this side
            planes = [Plane(origin + out * start, -out), Plane(facing.point - facing.normal * floor, -facing.normal)]
            frames += convex_toolpath(beam, planes + _beam_planes(beam), at, bit_diameter, stepdown, safe_height)[0]
            deepest = max(deepest, floor)
        if frames:
            note = f"{name} from side {at}, {deepest:.0f} mm deep, {len(frames)} frames"
            setups.append((at, frames, note + (f", LEFT STANDING: {left:.0f} mm, the bit is too short for this side" if left > 1e-6 else "")))
    return setups


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
    # at an angle to the beam, open at both sides, as in Group 3's pyramid: every line runs right through, parallel to the walls
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=120)
    for skew in (60.0, 86.087, 93.913, 120.0):
        beam.features = [Pocket(start_x=300, start_y=0, start_depth=5, length=105, width=80, internal_angle=skew)]
        (toolpath,) = toolpaths(beam)
        side = beam.ref_sides[0]
        cut = [side.to_local_coordinates(f.point) for f in toolpath.frames]
        for a, b in zip(cut, cut[1:]):
            if max(a[2], b[2]) > 1e-6:
                continue  # travel, above the wood
            if abs(b[1] - a[1]) > 20:
                assert abs(abs(b[1] - a[1]) - (80 + 2 * (r + CLEARANCE))) < 1e-6 and abs(abs((b[0] - a[0]) / (b[1] - a[1])) - abs(math.tan(math.radians(skew - 90)))) < 1e-6, (skew, a, b)
            else:  # from one line to the next, in the air
                assert min(a[1], b[1]) > 80 + r or max(a[1], b[1]) < -r, (skew, a, b)
        walls = [p for p, n in ((p, p.normal.dot(side.xaxis)) for p in _volume(beam, beam.features[0])) if abs(n) > 0.1]
        assert len(walls) == 2 and all(abs(max((f.point - p.point).dot(p.normal) for f in toolpath.frames) + r) < 0.11 for p in walls), skew  # a radius off both
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

    # Self-check: a tenon's setups, bit and spindle, never touch wood that stays, and together clear the rest past the shoulder, or say what they leave.
    import numpy as np

    def frame_axes(f):
        return np.array([list(f.xaxis), list(f.yaxis), list(f.normal)]), np.array(list(f.point))

    square = dict(angle=90.0, inclination=90.0, rotation=90.0, shape="square")
    for (w, h), count, leaves, tenon in (
        ((58, 58), 2, False, Tenon(orientation=OrientationType.END, start_x=960.0, start_y=29.0, start_depth=14.0, length=30.0, width=20.0, height=40.0, **square)),  # within reach
        ((80, 120), 4, False, Tenon(orientation=OrientationType.END, start_x=950.0, start_y=40.0, start_depth=30.0, length=60.0, width=20.0, height=40.0, **square)),  # 10 mm past its tip
        ((80, 120), 4, False, Tenon(orientation=OrientationType.END, start_x=960.0, start_y=40.0, start_depth=30.0, length=60.0, width=20.0, height=40.0, **square)),
        ((80, 120), 1, False, Tenon(orientation=OrientationType.START, start_x=40.0, start_y=23.0, start_depth=0.0, length=120.0, width=46.0, height=40.0, **square)),  # flush with three sides
        ((80, 120), 1, True, Tenon(orientation=OrientationType.START, start_x=40.0, start_y=15.0, start_depth=0.0, length=120.0, width=30.0, height=40.0, **square)),  # the bit is too short
    ):
        beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=w, height=h)
        beam.add_features(tenon)
        paths = toolpaths(beam)
        assert len(paths) == count and any("LEFT STANDING" in t.note for t in paths) == leaves, paths
        shoulder, pointing, (y0, y1), (d0, d1), end = _tenon_box(beam, tenon)
        grid = np.stack(np.meshgrid(np.arange(-40.0, abs(end - shoulder) + 1, 2.0), np.arange(1.1, w, 2.0), np.arange(1.1, h, 2.0), indexing="ij"), -1).reshape(-1, 3)
        s, y, d = grid.T
        stays = (s < 0) | ((s <= tenon.height) & (y > y0) & (y < y1) & (d > d0) & (d < d1))
        axes, point = frame_axes(beam.ref_sides[0])
        wood = np.c_[shoulder + pointing * s, y, -d] @ axes + point
        cleared = np.zeros(len(wood), bool)
        for t in paths:
            axes, point = frame_axes(beam.ref_sides[t.ref_side_index])
            local = (wood - point) @ axes.T
            tips = [(np.array(list(f.point)) - point) @ axes.T for f in t.frames]
            for a, b in zip(tips, tips[1:]):
                steps = max(1, int(np.linalg.norm(b - a) / 4))
                for k in range(steps + 1):
                    tip = a + (b - a) * k / steps
                    around, above = np.hypot(local[:, 0] - tip[0], local[:, 1] - tip[1]), local[:, 2] - tip[2]
                    bit = (around < BIT_DIAMETER / 2 - 0.05) & (above > 0.05)
                    assert not (bit & stays).any(), ("the bit cuts wood that stays", tenon, t)
                    for start, size in SPINDLE:
                        assert not ((around < size) & (above > start) & stays).any(), ("the spindle goes through wood that stays", tenon, t)
                    cleared |= (around < BIT_DIAMETER / 2 + 0.5) & (above > -0.5)
        corner = (s < 4.0) & ((np.abs(y - y0) < 4.0) | (np.abs(y - y1) < 4.0))  # rounded by the bit, in two setups
        standing = ~cleared & ~stays & (s > 3.0) & ~corner
        assert leaves or not standing.any(), ("left standing", (w, h), tenon.start_y, paths, grid[standing].min(0), grid[standing].max(0))
    print("Tenon setups clear around the tenon, and the spindle stays out of what stays.")

    # Self-check: birds mouths, half laps, planing and contour pockets never cut outside what they remove, and reach its bottom.
    from compas.geometry import Polyline
    from compas_timber.fabrication import Contour
    from wjim_lib import MillContour

    def cutting(beam, toolpath):  # (x, y, depth) below the side it's milled from, of every frame in the wood
        side = beam.ref_sides[toolpath.ref_side_index]
        return [(p[0], p[1], -p[2]) for p in (side.to_local_coordinates(f.point) for f in toolpath.frames) if -p[2] > 1e-6]

    r = BIT_DIAMETER / 2
    for notch in (BirdsMouth(start_x=300.0, start_depth=30.0, depth=30.0, width=80.0), BirdsMouth(start_x=300.0, start_depth=25.0, depth=25.0, width=80.0, inclination_1=60.0, inclination_2=150.0)):
        beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
        beam.add_features(notch)
        (toolpath,) = toolpaths(beam)
        faces = birds_mouth_planes(beam, notch)[:2]
        assert all((f.point - p.point).dot(p.normal) <= 1e-6 for f in toolpath.frames for p in faces)
        assert notch.start_depth - r - 3.0 < max(d for _, _, d in cutting(beam, toolpath)) <= notch.start_depth  # a flat bit stops short of the V's sharp bottom
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
    beam.add_features(LogHouseHalfLap(start_x=300.0, length=80.0, depth_ref_side=25.0, depth_opp_side=35.0))
    for toolpath, depth in zip(toolpaths(beam), (25.0, 35.0)):
        cut = cutting(beam, toolpath)
        assert max(d for _, _, d in cut) == depth and all(260 + r - 1e-6 <= x <= 340 - r + 1e-6 for x, _, _ in cut)
        assert min(y for _, y, _ in cut) < -r and max(y for _, y, _ in cut) > 80 + r  # through the width
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
    beam.add_features(Planing(start_x=200.0, length=100.0, depth=5.0, start_limited=True, end_limited=True, plane_side_2=False, plane_side_4=False))
    assert [t.ref_side_index for t in toolpaths(beam)] == [0, 2]
    for toolpath in toolpaths(beam):
        cut = cutting(beam, toolpath)
        assert max(d for _, _, d in cut) == 5.0 and all(200 + r - 1e-6 <= x <= 300 - r + 1e-6 for x, _, _ in cut)
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
    square = Polyline([[275, 15, 0], [325, 15, 0], [325, 65, 0], [275, 65, 0], [275, 15, 0]])
    beam.add_features([MillContour(Contour(square, 20.0), counter_sink=True, tool_position="left"), FreeContour(Contour(square, 10.0), tool_position="right")])
    pocket, outline = toolpaths(beam)
    cut = cutting(beam, pocket)
    assert max(d for _, _, d in cut) == 20.0 and all(275 + r - 1e-6 <= x <= 325 - r + 1e-6 and 15 + r - 1e-6 <= y <= 65 - r + 1e-6 for x, y, _ in cut)
    cut = cutting(beam, outline)
    assert max(d for _, _, d in cut) == 10.0 and all(abs(max(abs(x - 300), abs(y - 40)) - (25 + r)) < 1e-6 for x, y, _ in cut)  # on the square, a bit's radius out
    print("Birds mouths, half laps, planing and contours stay inside what they remove, and reach its bottom.")

    # Self-check: milled cuts stay in their offcut and reach through the beam, slots stay in the slot, a house leaves its nested tenon standing.
    from compas_timber.fabrication import DoubleCut, JackRafterCut, LongitudinalCut, Slot

    def offcut_toolpaths(cut):
        beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
        beam.add_features(cut)
        return beam, toolpaths(beam)

    for cut, count in (
        (JackRafterCut(orientation=OrientationType.END, start_x=560.0), 1),
        (JackRafterCut(orientation=OrientationType.END, start_x=540.0, angle=60.0, inclination=70.0), 1),
        (JackRafterCut(orientation=OrientationType.END, start_x=540.0, inclination=110.0), 1),
        (DoubleCut(orientation=OrientationType.END, start_x=560.0, start_y=40.0, angle_1=135.0, angle_2=45.0), 2),
        (DoubleCut(orientation=OrientationType.END, start_x=560.0, start_y=40.0, angle_1=45.0, angle_2=135.0), 1),
        (LongitudinalCut(start_y=20.0, inclination=45.0), 1),
    ):
        beam, paths = offcut_toolpaths(cut)
        assert len(paths) == count and all(t.frames and "UNDERCUT" not in t.note for t in paths), paths
        for planes, toolpath in zip(cut_volumes(beam, cut), paths):
            up = beam.ref_sides[toolpath.ref_side_index].normal
            assert all((f.point - p.point).dot(p.normal) <= -r * math.sqrt(1 - Vector(*p.normal).dot(up) ** 2) + 1e-6 for f in toolpath.frames for p in planes)  # the bit's disc in the offcut
            assert abs(max(d for _, _, d in cutting(beam, toolpath)) - 120.0 * (toolpath.ref_side_index in (0, 2)) - 80.0 * (toolpath.ref_side_index in (1, 3))) < 1e-6 or isinstance(cut, LongitudinalCut)
    beam, paths = offcut_toolpaths(JackRafterCut(orientation=OrientationType.END, start_x=540.0, inclination=70.0))
    assert paths[0].ref_side_index == 2, "a face leaning over the offcut is milled from the opposite side"
    # saw guides: a straight groove on each of two sides, through both edges, never in the part that stays and touching it
    for ref, cut in (
        (0, JackRafterCut(orientation=OrientationType.START, start_x=40.0)),
        (0, JackRafterCut(orientation=OrientationType.START, start_x=90.0, inclination=112.0)),
        (2, JackRafterCut(orientation=OrientationType.END, start_x=500.0, inclination=68.0)),
        (1, JackRafterCut(orientation=OrientationType.END, start_x=500.0, angle=60.0, inclination=70.0)),
        (0, JackRafterCut(orientation=OrientationType.START, start_x=160.0, angle=120.0, inclination=110.0)),
    ):
        cut.ref_side_index = ref
        beam, paths = offcut_toolpaths(cut)
        guides = saw_guides(beam, paths)
        assert [g.ref_side_index for g in guides] == [ref, {0: 1, 2: 1, 1: 0}[ref]], guides
        plane = cut.plane_from_params_and_beam(beam)
        for guide in guides:
            side = beam.ref_sides[guide.ref_side_index]
            width = beam.get_dimensions_relative_to_side(guide.ref_side_index)[0]
            tilt = math.sqrt(1 - Vector(*plane.normal).dot(side.normal) ** 2)
            assert abs(min((f.point - plane.point).dot(plane.normal) for f in guide.frames[1:3]) - r * tilt - max(0.0, -Vector(*plane.normal).dot(side.normal)) * GUIDE_DEPTH) < 1e-6, guide
            (xa, ya, za), (xb, yb, zb) = (side.to_local_coordinates(f.point) for f in guide.frames[1:3])
            assert len(guide.frames) == 4 and abs(za + GUIDE_DEPTH) < 1e-6 and abs(zb + GUIDE_DEPTH) < 1e-6
            assert abs(min(ya, yb) + r + CLEARANCE) < 1e-6 and abs(max(ya, yb) - width - r - CLEARANCE) < 1e-6, guide
    assert saw_guides(beam, toolpaths(beam, [])) == []
    beam, (toolpath,) = offcut_toolpaths(Slot(start_x=300.0, start_y=0.0, length=80.0, depth=50.0, thickness=10.0))
    assert saw_guides(beam, [toolpath]) == [toolpath]
    cut = cutting(beam, toolpath)
    assert all(abs(x - 300.0) <= 5 - r + 1e-6 for x, _, _ in cut) and abs(max(d for _, _, d in cut) - 50.0) < 1e-6
    beam, (toolpath,) = offcut_toolpaths(Slot(start_x=300.0, start_y=0.0, length=80.0, depth=50.0, thickness=4.0))
    assert not toolpath.frames
    nested = Tenon(orientation=OrientationType.END, start_x=580.0, start_y=40.0, start_depth=40.0, length=40.0, width=20.0, height=20.0, shape="square")
    house = House(orientation=OrientationType.END, start_x=560.0, start_y=40.0, start_depth=20.0, length=80.0, width=60.0, height=20.0, shape="square", subprocessings=[nested])
    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(600, 0, 0)), width=80, height=120)
    beam.add_features(house)
    for toolpath in toolpaths(beam):
        side = beam.ref_sides[0]
        for f in toolpath.frames:
            x, y, z = side.to_local_coordinates(f.point)
            assert not (580 - r + 1e-6 < x < 600 + r - 1e-6 and 30 - r + 1e-6 < y < 50 + r - 1e-6 and 40 + 1e-6 < -z < 80 - 1e-6), ("cuts the nested tenon", x, y, z)
    print("Cuts clear their offcuts, slots stay in the slot, a house leaves its nested tenon standing.")

    # Self-check: step joints, triangle and ridge/valley cuts and hip/valley notches only cut where compas_timber or wjim_lib remove wood.
    for cut in (
        StepJoint(orientation=OrientationType.END, start_x=520.0, strut_inclination=45.0, step_depth=20.0, heel_depth=0.0, step_shape=StepShapeType.STEP),
        StepJoint(start_x=80.0, strut_inclination=60.0, step_depth=0.0, heel_depth=20.0, step_shape=StepShapeType.TAPERED_HEEL),
        TriangleCut(start_x=300.0, start_y=40.0, start_depth=30.0, normal_1_x=0.707, normal_1_z=0.707, normal_2_x=-0.707, normal_2_z=0.707),
        RidgeValleyCut(start_y=30.0, start_depth=15.0, inclination_ref_side=20.0, inclination_opp_side=40.0),
        RidgeValleyCut(start_x=200.0, start_y=40.0, start_depth=20.0, inclination_ref_side=-30.0, inclination_opp_side=-30.0, start_limited=True, end_limited=True, length=200.0),
        HipValleyRafterNotch(start_x=300.0, start_y=40.0, start_depth=20.0, inclination=0.0),
        HipValleyRafterNotch(
            start_x=300.0, start_y=40.0, start_depth=20.0, inclination=0.0, angle_ref_edge=120.0, angle_opp_edge=120.0,
            width_counter_part_ref_edge_limited=True, width_counter_part_ref_edge=30.0, width_counter_part_opp_edge_limited=True, width_counter_part_opp_edge=30.0,
        ),
    ):
        beam, paths = offcut_toolpaths(cut)
        kept = beam.compute_modelgeometry()
        for toolpath in paths:
            side = beam.ref_sides[toolpath.ref_side_index]
            assert toolpath.frames and "UNDERCUT" not in toolpath.note, toolpath
            for f in toolpath.frames[:: max(1, len(toolpath.frames) // 100)]:
                tip = f.point + side.normal * 0.3  # just off the face it finishes
                disc = [tip + (side.xaxis * math.cos(a) + side.yaxis * math.sin(a)) * (r - 0.3) for a in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)]
                assert not any(kept.contains(p) for p in [tip] + disc), (type(cut).__name__, side.to_local_coordinates(f.point))

    # Self-check: text is engraved along compas_timber's strokes, MARK_DEPTH deep.
    text = Text("WJIM", start_x=150.0, start_y=40.0, text_height=60.0, alignment_vertical="center")
    beam, (toolpath,) = offcut_toolpaths(text)
    side, grooves = beam.ref_sides[0], text.grooves_for_element(beam)
    cut = [f.point for f in toolpath.frames if -side.to_local_coordinates(f.point)[2] > 1e-6]
    assert cut and all(abs(side.to_local_coordinates(p)[2] + MARK_DEPTH) < 1e-6 and any(g.contains_point(p, tol=1e-3) for g in grooves) for p in cut)
    print("Step joints, triangle, ridge and valley cuts and notches cut only what they remove; text follows its strokes.")
