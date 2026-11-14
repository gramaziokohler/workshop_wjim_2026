"""CornerNodeJoint: the joint for junction 0, where 3 beams meet (TOPO_Y).

Exported from Grasshopper by junction_lib. From here on this file is yours.
`corner_node.fixture.json`, next to it, is the situation it was exported from:
open this file in VS Code and the joint editor runs it against those beams.

The beams, in the order the constructor takes them:

- `edge_a`: beam 0 (category 'edge'), ends here
- `edge_b`: beam 2 (category 'edge'), ends here
- `post`: beam 4 (category 'post'), ends here

What it does:

- The post carries both edges. Its top is cut to the undersides of the two
  edges, so they sit on it (a double cut, one plane per edge).
- The edges cross over the post with a corner half-lap: each gives up half
  its depth where the other passes, so neither sits on top of the other.
  That part is an existing joint (`LLapJoint`), reused.
"""

from __future__ import annotations

from itertools import combinations

from compas.geometry import Point
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.elements import Beam

from compas.geometry import Plane
from compas_timber.connections import LLapJoint
from compas_timber.fabrication import DoubleCut

import debug


class CornerNodeJoint(Joint):
    """Joins `edge_a`, `edge_b`, `post`.

    Parameters
    ----------
    edge_a
        beam 0 (category 'edge'), ends here
    edge_b
        beam 2 (category 'edge'), ends here
    post
        beam 4 (category 'post'), ends here
    depth
        An example parameter - rename it, use it, or remove it.
    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_Y
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
        edge_a: Beam | None = None,
        edge_b: Beam | None = None,
        post: Beam | None = None,
        depth: float = 20.0,
        **kwargs,
    ) -> None:
        super().__init__(elements=(edge_a, edge_b, post), **kwargs)
        self.depth = depth

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
        """The edges reach past the post to the outer corner, as the lap needs."""
        self.lap.add_extensions()

    @property
    def lap(self) -> LLapJoint:
        """The two edges cross with a corner half-lap: an existing joint, reused."""
        return LLapJoint(self.edge_a, self.edge_b, cut_plane_bias=0.5)

    def underside(self, edge: Beam) -> Plane:
        """The face of `edge` the post carries."""
        face = edge.ref_sides[self.side_facing(edge, self.post)]
        return Plane(face.point, face.normal)

    def add_features(self) -> None:
        """Lap the edges into each other, and cut the post to sit under both."""
        lap = self.lap
        lap.add_features()
        self.features.extend(lap.features)

        planes = [self.underside(self.edge_a), self.underside(self.edge_b)]
        seat = DoubleCut.from_planes_and_beam(planes, self.post)
        self.post.add_features(seat)
        self.features.append(seat)
        for plane in planes:
            debug.show(plane, "underside")
