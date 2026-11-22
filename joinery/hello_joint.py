"""HelloJoint: an L butt joint, for junction 0, where 2 beams meet (TOPO_L).

Exported from Grasshopper by junction_lib. From here on this file is yours.
`hello_joint.fixture.json`, next to it, is the situation it was exported from:
open this file in VS Code and the joint editor runs it against those beams.

The beams, in the order the constructor takes them:

- `main_beam`: beam 0 (category 'main_beam'), ends here
- `cross_beam`: beam 10 (category 'cross_beam'), ends here

In carpentry words: the two beams meet at a corner. The main beam is cut square
against the side of the cross beam and sits `mill_depth` deep in a lap milled
into it. The cross beam carries: it runs on past the main beam and is cut flush
with the main beam's far side, so the corner is closed.

Uncomment the steps one at a time, top to bottom:

1-3. Look before cutting: where the beams meet, which way each one runs, and
     which side of each faces the other.
4.   Cut the main beam against the cross beam.
5-6. The cross beam ends at the joint's centre, so half the main beam hangs out
     past it: extend the cross beam to the main beam's far side, then cut it there.
7.   Mill a lap into the cross beam and let the main beam reach into it.

Then, in Grasshopper, switch `everywhere` on: the same joint on every pair.
"""

from __future__ import annotations

from itertools import combinations

from compas.geometry import Line
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.elements import Beam

from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap

import debug


class HelloJoint(Joint):
    """An L butt joint: `main_beam` is cut against `cross_beam`, which runs on past it.

    Parameters
    ----------
    main_beam
        beam 0 (category 'main_beam'), ends here and is cut against the cross beam
    cross_beam
        beam 10 (category 'cross_beam'), ends here, extended past the main beam
    mill_depth
        How deep the main beam sits in the cross beam, in mm.
    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_L
    MIN_ELEMENT_COUNT = 2
    MAX_ELEMENT_COUNT = 2

    @property
    def __data__(self) -> dict:
        data = super().__data__
        data["mill_depth"] = self.mill_depth
        return data

    # Annotated parameters can be set from the Fixture's "params".
    def __init__(
        self,
        main_beam: Beam | None = None,
        cross_beam: Beam | None = None,
        mill_depth: float = 20.0,
        **kwargs,
    ) -> None:
        super().__init__(elements=(main_beam, cross_beam), **kwargs)
        self.mill_depth = mill_depth

    @property
    def main_beam(self) -> Beam:
        return self.elements[0]

    @property
    def cross_beam(self) -> Beam:
        return self.elements[1]

    @property
    def location(self) -> Point:
        """Where the beams meet: the centre of the closest points between their centerlines."""
        if self._location is None:
            points = []
            for beam_a, beam_b in combinations(self.elements, 2):
                point_a, point_b = intersection_line_line(
                    beam_a.centerline, beam_b.centerline
                )
                if point_a is not None:
                    points.extend([point_a, point_b])
            self._location = Point(*centroid_points(points))
        return self._location

    def side_index_facing(self, beam: Beam, other: Beam) -> int:
        """The index of the reference side of `beam` that faces `other`.

        `beam_ref_side_incidence(a, b)` measures b's sides against a's centerline,
        so `beam` goes second: the side facing `other` is the one at the smallest angle.
        """
        incidence = beam_ref_side_incidence(other, beam, ignore_ends=True)
        return min(incidence, key=incidence.get)

    def butt_plane(self) -> Plane:
        """Where the main beam is cut: the cross beam's facing side, sunk `mill_depth` into it.

        Its normal points into the cross beam, so the cut keeps the part of the main beam outside it.
        """
        face = self.cross_beam.ref_sides[
            self.side_index_facing(self.cross_beam, self.main_beam)
        ]
        return Plane(face.point - face.normal * self.mill_depth, -face.normal)

    def back_plane(self) -> Plane:
        """Where the cross beam is cut: the main beam's far side, the one opposite the side facing the cross beam."""
        return Plane.from_frame(
            self.main_beam.opp_side(
                self.side_index_facing(self.main_beam, self.cross_beam)
            )
        )

    def add_extensions(self) -> None:
        """Lengthen beam blanks that must reach past their centerline end. Runs before `add_features()`."""
        pass

        # STEP 5: the cross beam ends at the joint's centre: reach on to the main beam's far side.
        # start, end = self.cross_beam.extension_to_plane(self.back_plane())
        # self.cross_beam.add_blank_extension(start, end, str(self.guid))

        # STEP 7: the main beam reaches into the lap, as deep as the butt plane.
        # An extension only ever lengthens a blank: a negative one is dropped. On a square L the
        # main beam already runs to the cross beam's centre, past the butt plane, so nothing changes
        # until mill_depth is more than half the cross beam (try 60) or the main beam comes in sloped.
        # start, end = self.main_beam.extension_to_plane(self.butt_plane())
        # self.main_beam.add_blank_extension(start, end, str(self.guid))

    def add_features(self) -> None:
        """Add this joint's processings to its beams. Runs during `process_joinery()`."""
        main = self.main_beam
        cross = self.cross_beam

        # STEP 1: where the beams meet.
        # debug.show(self.location, "location")

        # STEP 2: each beam's centerline, where it starts, and which way it leaves the joint.
        # A beam's start decides its reference sides: flip a beam and its sides renumber.
        # Despite its name, `get_beam_direction_towards_joint` points from the joint out along the beam.
        # debug.show(main.centerline, "main_beam centerline")
        # debug.show(main.centerline.start, "main_beam start")
        # main_direction = self.get_beam_direction_towards_joint(main)
        # debug.show(Line(self.location, self.location + main_direction * 300), "main_beam leaves the joint")
        #
        # debug.show(cross.centerline, "cross_beam centerline")
        # debug.show(cross.centerline.start, "cross_beam start")
        # cross_direction = self.get_beam_direction_towards_joint(cross)
        # debug.show(Line(self.location, self.location + cross_direction * 300), "cross_beam leaves the joint")

        # STEP 3: the side of each beam that faces the other, and the main beam's far side.
        # main_side = self.side_index_facing(main, cross)
        # cross_side = self.side_index_facing(cross, main)
        # debug.show(main.ref_sides[main_side], f"main_beam side {main_side} faces cross_beam")
        # debug.show(cross.ref_sides[cross_side], f"cross_beam side {cross_side} faces main_beam")
        # debug.show(main.opp_side(main_side), "main_beam far side")

        # Steps 4-7 use main_side and cross_side from STEP 3.
        # STEP 4: cut the main beam against the cross beam. mill_depth is 0 in the Fixture, so it's cut flush.
        # butt = JackRafterCut.from_plane_and_beam(self.butt_plane(), main, main_side)
        # main.add_features(butt)
        # self.features.append(butt)

        # STEP 6: with STEP 5's extension on, cut the cross beam flush with the main beam's far side.
        # back = JackRafterCut.from_plane_and_beam(self.back_plane(), cross, cross_side)
        # cross.add_features(back)
        # self.features.append(back)

        # STEP 7: mill the lap the main beam sits in: as wide as the main beam, mill_depth deep.
        # Set mill_depth to 20 in the Fixture's "params": the butt cut follows it down.
        # lap_width = main.get_dimensions_relative_to_side(main_side)[1]
        # mill_depth = self.mill_depth
        # mill_depth = 7.3  # 0.00..10.0

        # lap = Lap.from_plane_and_beam(
        #     main.ref_sides[main_side],
        #     cross,
        #     lap_width,
        #     mill_depth,
        #     ref_side_index=cross_side,
        # )
        # cross.add_features(lap)
        # self.features.append(lap)
