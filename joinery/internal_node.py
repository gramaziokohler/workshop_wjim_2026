"""InternalNodeJoint: the joint for junction 2, where 3 beams meet (TOPO_K).

Exported from Grasshopper by junction_lib. From here on this file is yours.
`internal_node.fixture.json`, next to it, is the situation it was exported from:
open this file in VS Code and the joint editor runs it against those beams.

The beams, in the order the constructor takes them:

- `post`: beam 8 (category 'post'), ends here
- `rafter`: beam 11 (category 'rafter'), passes through
- `purlin`: beam 16 (category 'purlin'), passes through

Before writing any code, write down here what this joint should do, in
carpentry words: which beam is cut, which one carries, what each cut is for.
"""

from __future__ import annotations

from itertools import combinations

from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Frame
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.elements import Beam

# Your building blocks are BTLx processings, for example:
# from compas_timber.fabrication import Lap, Pocket, Drilling, JackRafterCut
from compas_timber.fabrication import JackRafterCut
from compas_timber.fabrication import Lap

import debug


class InternalNodeJoint(Joint):
    """Joins `post`, `rafter`, `purlin`.

    Parameters
    ----------
    post
        beam 8 (category 'post'), ends here
    rafter
        beam 11 (category 'rafter'), passes through
    purlin
        beam 16 (category 'purlin'), passes through
    depth
        An example parameter - rename it, use it, or remove it.
    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_K
    MIN_ELEMENT_COUNT = 3
    MAX_ELEMENT_COUNT = 3

    @property
    def __data__(self) -> dict:
        data = super().__data__
        data["depth"] = self.depth
        return data

    # Annotated parameters can be set from the Fixture's "params".
    def __init__(
        self,
        post: Beam | None = None,
        rafter: Beam | None = None,
        purlin: Beam | None = None,
        depth: float = 20.0,
        **kwargs,
    ) -> None:
        super().__init__(elements=(post, rafter, purlin), **kwargs)
        self.depth = depth

    @property
    def post(self) -> Beam:
        return self.elements[0]

    @property
    def rafter(self) -> Beam:
        return self.elements[1]

    @property
    def purlin(self) -> Beam:
        return self.elements[2]

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

    def side_facing(self, beam: Beam, other: Beam) -> Frame:
        """The index of `beam`'s reference side that faces `other`."""
        incidence = beam_ref_side_incidence(other, beam, ignore_ends=True)
        index = min(incidence, key=incidence.get)
        return beam.ref_sides[index]

    def add_extensions(self) -> None:
        """Lengthen beam blanks that must reach past their centerline end. Runs before `add_features()`."""
        pass

    def add_features(self) -> None:
        """Add this joint's processings to its beams. Runs during `process_joinery()`."""
        # Each beam points from the junction back along its centerline.
        side = self.side_facing(self.rafter, self.post)
        debug.show(side, "rafter side facing post")

        cut = JackRafterCut.from_plane_and_beam(side.flipped(), self.post)

        depth = 20.0  # 0.00..30.0
        length = 100.0

        from compas.geometry import Plane

        lap = Lap.from_plane_and_beam(
            Plane.from_frame(side), self.rafter, length=length, depth=depth
        )

        self.post.add_feature(cut)
