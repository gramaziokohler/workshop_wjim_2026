"""DayNightNodeJoint: a corner that tells the time. For junctions where 3 beams meet (TOPO_Y).

`day_night_node.fixture.json`, next to it, holds the beams of `corner_node`.

The beams, in the order the constructor takes them:

- `edge_a`: the day edge, ends here
- `edge_b`: the night edge, ends here
- `post`: ends here

What makes sense:

- The post carries. It runs up past both edges, and each edge sits `housing_depth`
  deep in it with its whole section (`HouseMortise`), cut to a level seat so it
  bears on the post and can't slide down its own slope.
- From the housing a blind tenon reaches on into the post (`Mortise` in the post,
  `Pocket`s on the edge). The two tenons cross in the post's heart: the day edge's
  passes above, the night edge's below.
- The bit leaves mortise corners round, so each gets a relief hole (`Drilling`), and
  the edges' arrises are chamfered (`Chamfer`) to clear the housing's round corners.
- A draw-bored peg locks each tenon: the hole in the tenon is `drawbore` closer to
  its shoulder than the holes in the post, so driving the peg in pulls the edge home.

What makes less sense:

- Each peg comes out of the post's outside and is left long. On the day face it is
  the gnomon of a sundial: hour lines for `latitude`, three of them deep (`Pocket`),
  the others engraved (`Marking`). It only keeps time on a wall that faces south.
- On the night face the other peg is Polaris. The Big Dipper turns below it, its
  holes (`Drilling`) as large as its stars are bright.
- The day edge carries XIX (`Text`), the hour it hands over to the night, the night edge a waxing moon (`MillContour`).
- Under both edges, notches grow along the Fibonacci numbers (`Lap`).
- The post's head is a neck (`LogHouseHalfLap`, twice) under a pyramid (`DoubleCut`, twice),
  and its arrises are stop-chamfered below the joint.
"""

from __future__ import annotations

import math
from itertools import combinations

from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Line
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Polyline
from compas.geometry import Vector
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas.geometry import intersection_line_plane
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.connections.utilities import beam_ref_side_incidence_with_vector
from compas_timber.elements import Beam
from compas_timber.errors import BeamJoiningError
from compas_timber.fabrication import DoubleCut
from compas_timber.fabrication import Drilling
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap
from compas_timber.fabrication import Mortise
from compas_timber.fabrication import Pocket
from compas_timber.fabrication import Text
from wjim_lib import Chamfer
from wjim_lib import HouseMortise
from wjim_lib import LogHouseHalfLap
from wjim_lib import Marking
from wjim_lib import MillContour

import debug

# distance from the pole and right ascension in degrees, magnitude: Dubhe, Merak, Phecda, Megrez, Alioth, Mizar, Alkaid
BIG_DIPPER = ((28.2, 165.9, 1.8), (33.6, 165.5, 2.3), (36.3, 178.5, 2.4), (33.0, 183.9, 3.3), (34.0, 193.5, 1.8), (35.1, 201.0, 2.2), (40.7, 206.9, 1.9))
FIBONACCI = (8.0, 13.0, 21.0, 34.0)


class DayNightNodeJoint(Joint):
    """Joins `edge_a`, `edge_b`, `post`.

    Parameters
    ----------
    edge_a
        The day edge, ends here. Its tenon passes above.
    edge_b
        The night edge, ends here. Its tenon passes below.
    post
        Ends here, carries both.
    housing_depth
        How deep each edge sits in the post with its whole section.
    tenon_length
        How deep each tenon reaches into the post, from its face.
    tenon_width
        How thick each tenon is.
    tenon_height
        How tall each tenon is.
    peg_diameter
        The peg through each tenon.
    drawbore
        How much closer to the shoulder the peg's hole is in the tenon than in the post.
    crown_height
        How far the post reaches past the edges.
    chamfer_depth
        The chamfer on every arris, on the post and the edges.
    latitude
        Where the sundial stands. Chiayi: 23.5, on the Tropic of Cancer.
    sky_scale
        How many mm on the post are a degree in the sky.
    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_Y
    MIN_ELEMENT_COUNT = 3
    MAX_ELEMENT_COUNT = 3

    @property
    def __data__(self) -> dict:
        data = super().__data__
        data["housing_depth"] = self.housing_depth
        data["tenon_length"] = self.tenon_length
        data["tenon_width"] = self.tenon_width
        data["tenon_height"] = self.tenon_height
        data["peg_diameter"] = self.peg_diameter
        data["drawbore"] = self.drawbore
        data["crown_height"] = self.crown_height
        data["chamfer_depth"] = self.chamfer_depth
        data["latitude"] = self.latitude
        data["sky_scale"] = self.sky_scale
        return data

    # Annotated parameters can be set from the Fixture's "params".
    def __init__(
        self,
        edge_a: Beam | None = None,
        edge_b: Beam | None = None,
        post: Beam | None = None,
        housing_depth: float = 10.0,
        tenon_length: float = 70.0,
        tenon_width: float = 30.0,
        tenon_height: float = 50.0,
        peg_diameter: float = 12.0,
        drawbore: float = 1.5,
        crown_height: float = 90.0,
        chamfer_depth: float = 8.0,
        latitude: float = 23.5,
        sky_scale: float = 3.0,
        **kwargs,
    ) -> None:
        super().__init__(elements=(edge_a, edge_b, post), **kwargs)
        self.housing_depth = housing_depth
        self.tenon_length = tenon_length
        self.tenon_width = tenon_width
        self.tenon_height = tenon_height
        self.peg_diameter = peg_diameter
        self.drawbore = drawbore
        self.crown_height = crown_height
        self.chamfer_depth = chamfer_depth
        self.latitude = latitude
        self.sky_scale = sky_scale

    @property
    def edge_a(self) -> Beam:
        return self.elements[0]

    @property
    def edge_b(self) -> Beam:
        return self.elements[1]

    @property
    def post(self) -> Beam:
        return self.elements[2]

    @property
    def edges(self) -> tuple[Beam, Beam]:
        return self.edge_a, self.edge_b

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

    @property
    def up(self) -> Vector:
        """Along the post, out of its foot."""
        return Vector(*(self.location - self.post.centerline.midpoint)).unitized()

    def outward(self, beam: Beam) -> Vector:
        """Along `beam`, away from the joint."""
        return Vector(*(beam.centerline.midpoint - self.location)).unitized()

    def side_facing(self, beam: Beam, other: Beam) -> int:
        """The index of the reference side of `beam` that faces `other`."""
        incidence = beam_ref_side_incidence(other, beam, ignore_ends=True)
        return min(incidence, key=incidence.get)

    def side_towards(self, beam: Beam, vector) -> int:
        """The index of `beam`'s reference side whose normal points most along `vector`."""
        angles = beam_ref_side_incidence_with_vector(beam, vector, ignore_ends=True)
        return min(angles, key=angles.get)

    def hit(self, point, vector, side: Frame) -> Point:
        """Where the line from `point` along `vector` meets the plane of `side`."""
        return Point(*intersection_line_plane(Line(point, point + vector), Plane.from_frame(side)))

    def mouth(self, edge: Beam) -> Frame:
        """Where `edge`'s centerline enters the post: X across the post's face, Y up, Z out of the face."""
        face = self.post.ref_sides[self.side_facing(self.post, edge)]
        point = self.hit(edge.centerline.midpoint, self.outward(edge), face)
        return Frame(point, self.up.cross(face.normal), self.up)

    def housing_range(self, edge: Beam, mouth: Frame) -> tuple[float, float]:
        """Bottom and top of the housing, above `mouth`: what the sloped edge fills at every depth of it."""
        depths = (mouth.point, mouth.point - mouth.normal * self.housing_depth)
        lows, highs = [], []
        for point in depths:
            for side, found in ((self.side_towards(edge, -self.up), lows), (self.side_towards(edge, self.up), highs)):
                found.append((self.hit(point, self.up, edge.ref_sides[side]) - mouth.point).dot(self.up))
        return max(lows), min(highs)

    def crown_base(self) -> Point:
        """The point on the post's axis up to which the edges cover it."""
        tops = [edge.ref_sides[self.side_towards(edge, self.up)] for edge in self.edges]
        reach = max(max(self.post.extension_to_plane(top)) for top in tops)
        return self.post.endpoint_closest_to_point(self.location)[1] + self.up * reach

    def add_extensions(self) -> None:
        """The post reaches up to its crown; each edge reaches through the post, for its tenon to be cut from."""
        end, point = self.post.endpoint_closest_to_point(self.location)
        reach = (self.crown_base() - point).dot(self.up) + self.crown_height + 5.0  # stock past the pyramid's tip
        self.post.add_blank_extension(reach * (end == "start"), reach * (end == "end"), self.guid)
        for edge in self.edges:
            far_face = self.post.opp_side(self.side_facing(self.post, edge))
            edge.add_blank_extension(*edge.extension_to_plane(far_face), self.guid)

    def add_features(self) -> None:
        """Join first, decorate after."""
        self.features = []
        mouths = [self.mouth(edge) for edge in self.edges]
        levels = [(mouth.point - self.location).dot(self.up) for mouth in mouths]
        level = 0.5 * sum(levels)  # where one tenon passes over the other
        clear = 12.0  # wood left between the tenons, 6.0..20.0

        pins = []
        for edge, mouth, own_level, over in zip(self.edges, mouths, levels, (1, -1)):
            tenon_middle = level - own_level + over * 0.5 * (self.tenon_height + clear)
            pins.append(self.join(edge, mouth, tenon_middle))

        # each peg comes out of the post opposite the other edge
        self.decorate_day_face(pins[1], mouths[0].normal)
        self.decorate_night_face(pins[0], mouths[1].normal)
        self.decorate_edge(self.edge_a, self.edge_b)
        self.decorate_edge(self.edge_b, self.edge_a)
        self.decorate_post()

    def add(self, beam: Beam, *processings) -> None:
        beam.add_features(list(processings))
        self.features.extend(processings)

    # ---------------------------------------------------------------- what makes sense

    def join(self, edge: Beam, mouth: Frame, middle: float) -> Point:
        """House, tenon and peg `edge` in the post. Returns where the peg crosses the tenon's middle plane."""
        post = self.post
        post_side = self.side_facing(post, edge)
        up, out, across = self.up, mouth.normal, mouth.xaxis
        cheeks = [self.side_towards(edge, across * sign) for sign in (1, -1)]
        width, _ = edge.get_dimensions_relative_to_side(self.side_towards(edge, up))
        _, post_thickness = post.get_dimensions_relative_to_side(post_side)
        bottom, top = self.housing_range(edge, mouth)
        tenon_bottom, tenon_top = middle - 0.5 * self.tenon_height, middle + 0.5 * self.tenon_height
        if tenon_bottom < bottom or tenon_top > top:
            raise BeamJoiningError([edge, post], self, "The tenons don't fit in the edges' height: lower tenon_height.", [mouth])
        if 0.5 * post_thickness + self.peg_diameter > self.tenon_length or self.tenon_length > post_thickness:
            raise BeamJoiningError([edge, post], self, "The tenon must reach a peg's diameter past the post's middle, and not through it: change tenon_length.", [mouth])

        # the post: a housing, the mortise in its floor, and a relief hole in each of the mortise's corners
        house = HouseMortise.from_frame_and_beam(
            Frame(mouth.point + up * bottom, across, up), post, length=top - bottom, width=width, depth=self.housing_depth, shape="square", ref_side_index=post_side
        )
        mortise = Mortise.from_frame_and_beam(
            Frame(mouth.point + up * tenon_bottom, across, up),
            post,
            start_depth=self.housing_depth,
            length=self.tenon_height,
            width=self.tenon_width,
            depth=self.tenon_length - self.housing_depth,
            shape="square",
            ref_side_index=post_side,
        )
        house.add_subprocessing(mortise)
        self.add(post, house)
        relief = 8.0  # diameter of the corner holes, 6.0..14.0
        inset = 0.5 * relief / math.sqrt(2.0)  # the hole's rim passes through the corner
        for side in (-1, 1):
            for height in (tenon_bottom + inset, tenon_top - inset):
                corner = mouth.point + across * side * (0.5 * self.tenon_width - inset) + up * height
                self.add(post, Drilling.from_line_and_element(Line(corner + out, corner - out * self.tenon_length), post, relief))

        # the edge: cut off at the tenon's tip, then everything that isn't tenon or housed stub, half from each cheek
        half, tip = 0.5 * width, mouth.point + out * -self.tenon_length
        for at, side, low, high, label in ((mouth.point, half, bottom, top, "housing"), (tip, 0.5 * self.tenon_width, tenon_bottom, tenon_top, "tenon tip")):
            corners = [at + across * (side * x) + up * z for x, z in ((-1, low), (1, low), (1, high), (-1, high))]
            debug.show(Polyline(corners + corners[:1]), label)
        self.add(edge, JackRafterCut.from_plane_and_beam(Plane(tip, -out), edge))
        beyond = 40.0  # how far the pockets reach past the edge, so they are open there
        shoulder, past = self.housing_depth, self.tenon_length + beyond
        low, high = bottom - 2 * beyond, top + 2 * beyond
        rests = (  # (from depth, to depth), (from, to) beside the middle, (from, to) above the mouth
            ((shoulder, past), (0.5 * self.tenon_width, half), (low, high)),  # cheek
            ((shoulder, past), (0.0, half), (tenon_top, high)),  # over the tenon
            ((shoulder, past), (0.0, half), (low, tenon_bottom)),  # under it
            ((0.0, shoulder), (0.0, half), (top, high)),  # the stub's level top
            ((0.0, shoulder), (0.0, half), (low, bottom)),  # its level seat
        )
        for sign, cheek in zip((1, -1), cheeks):
            for (d0, d1), (w0, w1), (h0, h1) in rests:
                centre = mouth.point - out * (0.5 * (d0 + d1)) + across * (sign * 0.5 * (w0 + w1)) + up * (0.5 * (h0 + h1))
                box = Box(w1 - w0, h1 - h0, d1 - d0, Frame(centre, across, up))
                self.add(edge, Pocket.from_volume_and_element(box.to_polyhedron(), edge, ref_side_index=cheek))

        # the peg, from the post's outside: opposite the other edge's mouth
        other = self.edge_b if edge is self.edge_a else self.edge_a
        outside = -self.mouth(other).normal
        pin = mouth.point + out * (-0.5 * post_thickness) + up * middle
        peg = Line(self.hit(pin, outside, post.ref_sides[self.side_towards(post, outside)]) + outside, pin - outside * self.tenon_width)
        debug.show(Line(pin - outside * self.tenon_width, pin + outside * post_thickness), "peg")
        self.add(post, Drilling.from_line_and_element(peg, post, self.peg_diameter))
        drawn = pin + out * self.drawbore
        self.add(edge, Drilling.from_line_and_element(Line(drawn + outside * width, drawn - outside * width), edge, self.peg_diameter))
        return pin

    # ---------------------------------------------------------------- what makes less sense

    def outside_face(self, pin: Point, out) -> tuple[int, Frame]:
        """The post's side whose normal is -`out`, and a frame on it where the peg at `pin` comes out: X right, Y up."""
        index = self.side_towards(self.post, out * -1)
        normal = self.post.ref_sides[index].normal
        return index, Frame(self.hit(pin, normal, self.post.ref_sides[index]), self.up.cross(normal), self.up)

    def decorate_day_face(self, pin: Point, out) -> None:
        """A sundial around the peg: the hours from 7 to 17, 9, 12 and 15 deeper."""
        index, dial = self.outside_face(pin, out)
        width, _ = self.post.get_dimensions_relative_to_side(index)
        margin = 8.0  # between the hour lines and the post's arris, 4.0..15.0
        noon = 95.0  # length of the line of noon, 60.0..120.0
        for hour in range(7, 18):
            turn = math.radians(15.0 * (hour - 12))
            angle = math.atan(math.cos(math.radians(self.latitude)) * math.tan(turn))  # a vertical dial facing south
            ray = dial.xaxis * math.sin(angle) - dial.yaxis * math.cos(angle)
            end = min(noon, (0.5 * width - margin) / max(abs(math.sin(angle)), 1e-9))
            deep = hour % 3 == 0
            line = Line(dial.point + ray * (24.0 if deep else 34.0), dial.point + ray * end)
            debug.show(line, f"{hour}h")
            if deep:
                box = Box(line.length, 7.0, 6.0, Frame(line.midpoint, ray, dial.normal.cross(ray)))  # half of it in the wood: 3 mm
                self.add(self.post, Pocket.from_volume_and_element(box.to_polyhedron(), self.post, ref_side_index=index))
            else:
                self.add(self.post, Marking.from_line_and_element(line, self.post, ref_side_index=index))

    def decorate_night_face(self, pin: Point, out) -> None:
        """The Big Dipper below the peg, which is Polaris."""
        _, sky = self.outside_face(pin, out)
        middle = 0.5 * (BIG_DIPPER[0][1] + BIG_DIPPER[-1][1])  # this right ascension hangs straight down
        depth = 8.0  # of the stars, 3.0..15.0
        stars = []
        for distance, ascension, magnitude in BIG_DIPPER:
            turn = math.radians(ascension - middle)
            star = sky.point + (sky.xaxis * math.sin(turn) - sky.yaxis * math.cos(turn)) * (distance * self.sky_scale)
            stars.append(star)
            diameter = max(6.0, 15.0 - 2.5 * magnitude)
            self.add(self.post, Drilling.from_line_and_element(Line(star + sky.normal, star - sky.normal * depth), self.post, diameter))
        debug.show(Polyline(stars[3:4] + stars), "Big Dipper")
        debug.show(Line(stars[1], sky.point), "to Polaris")

    def decorate_edge(self, edge: Beam, other: Beam) -> None:
        """Chamfers at the joint, Fibonacci notches underneath, and XIX or the moon on the outer cheek."""
        mouth, along = self.mouth(edge), self.outward(edge)
        index = self.side_towards(edge, -self.outward(other))
        cheek = edge.ref_sides[index]
        cheek_height, _ = edge.get_dimensions_relative_to_side(index)

        chamfered = 120.0  # how far the chamfers run from the post, 40.0..200.0
        self.chamfer(edge, mouth.point, along, chamfered, to_the_end=True)

        under = self.side_towards(edge, -self.up)
        notch = mouth.point + along * (chamfered + 20.0)
        for length in FIBONACCI:
            self.add(edge, Lap.from_plane_and_beam(Plane(notch + along * length, along), edge, length, 6.0, ref_side_index=under))
            notch = notch + along * (length + 13.0)

        start = self.hit(mouth.point, cheek.normal, cheek)
        rightward = self.up.cross(cheek.normal)
        if edge is self.edge_a:
            at = cheek.to_local_coordinates(start + along * 250.0)
            self.add(
                edge,
                Text(
                    "XIX",  # reads the same upside down (compas_timber 3.0.2 ignores Text's angle), no curves (each is dozens of cuts, a Run has 2 s)
                    start_x=at.x,
                    start_y=at.y,
                    alignment_vertical="center",
                    alignment_horizontal="center",
                    text_height=60.0,
                    ref_side_index=index,
                ),
            )
            return
        radius = 0.17 * cheek_height
        right = along if along.dot(rightward) > 0 else along * -1
        for i, phase in enumerate((0.0, 0.5, 1.0)):  # first quarter, gibbous, full
            centre = start + along * (150.0 + i * 2.5 * radius)
            turns = [math.radians(a) for a in range(-90, 270, 10)]
            points = [centre + right * (radius * math.cos(t) * (1.0 if math.cos(t) > 0 else phase)) + cheek.normal.cross(right) * (radius * math.sin(t)) for t in turns]
            if not phase:
                points = points[:19]  # the terminator is straight
            moon = Polyline(points + points[:1])
            debug.show(moon, "moon")
            self.add(edge, MillContour.from_polyline_and_element(moon, edge, depth=4.0, interior=True, ref_side_index=index))

    def decorate_post(self) -> None:
        """A neck, a pyramid, and stop-chamfers below the joint."""
        post, up, base = self.post, self.up, self.crown_base()
        apex = base + up * self.crown_height
        for index in (0, 1):
            side = post.ref_sides[index]
            _, width = post.get_dimensions_relative_to_side(index)  # square to it
            neck = side.to_local_coordinates(base + up * 19.0)
            self.add(post, LogHouseHalfLap(start_x=neck.x, length=14.0, depth_ref_side=8.0, depth_opp_side=8.0, ref_side_index=index))
            pitch = math.radians(45.0)
            # normals into the post, and each plane's point on its own facet: DoubleCut sorts the planes by their points
            planes = []
            for sign in (1, -1):
                sideways = side.normal * sign
                foot = apex + sideways * (0.5 * width) - up * (0.5 * width * math.tan(pitch))
                planes.append(Plane(foot, up * -math.cos(pitch) - sideways * math.sin(pitch)))
            debug.show(Line(planes[0].point, apex), "crown")
            self.add(post, DoubleCut.from_planes_and_beam(planes, post))
        self.chamfer(post, self.location + up * -160.0, -up, 200.0)  # not `-`: Point minus Vector is a Vector

    def chamfer(self, beam: Beam, point: Point, along, length: float, to_the_end=False) -> None:
        """Chamfer `beam`'s arrises from `point` on, `length` along `along`. `to_the_end`: out of the joint's end as well."""
        a, b = (beam.ref_sides[0].to_local_coordinates(p).x for p in (point, point + along * length))
        start, end = (not to_the_end or a > b), (not to_the_end or a < b)
        limits = {"start_x": min(a, b), "length": abs(b - a), "depth": self.chamfer_depth, "start_limited": start, "end_limited": end}
        self.add(beam, Chamfer(chamfer_edge_23=False, chamfer_edge_34=False, ref_side_index=0, **limits))
        self.add(beam, Chamfer(chamfer_edge_12=False, chamfer_edge_41=False, ref_side_index=2, **limits))
