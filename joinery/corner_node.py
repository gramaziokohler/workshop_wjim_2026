"""CornerNodeJoint: the joint for junction 0, where 3 beams meet (TOPO_Y).

Exported from Grasshopper by junction_lib. From here on this file is yours.
`corner_node.fixture.json`, next to it, is the situation it was exported from:
open this file in VS Code and the joint editor runs it against those beams.

The beams, in the order the constructor takes them:

- `edge_a`: beam 0 (category 'edge'), ends here
- `edge_b`: beam 2 (category 'edge'), ends here
- `post`: beam 4 (category 'post'), ends here

What it does:

- The post runs up past both edges, so each edge has a face of the post to
  butt into.
- Each edge butts into its face of the post with a step: a shallow notch in
  the post (`StepJointNotch`) that the edge's end (`StepJoint`) sits in, so
  the post carries the edge rather than a nail.
"""

from __future__ import annotations

from itertools import combinations

from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import centroid_points
from compas.geometry import intersection_line_line
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.connections.utilities import beam_ref_side_incidence
from compas_timber.connections.utilities import beam_ref_side_incidence_with_vector
from compas_timber.elements import Beam
from compas_timber.fabrication import StepJoint
from compas_timber.fabrication import StepJointNotch

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
    step_depth
        How deep each edge steps into the post.
    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_Y
    MIN_ELEMENT_COUNT = 3
    MAX_ELEMENT_COUNT = 3

    @property
    def __data__(self) -> dict:
        data = super().__data__
        data["step_depth"] = self.step_depth
        return data

    # Annotated parameters can be set from the Fixture's "params".
    def __init__(
        self,
        edge_a: Beam | None = None,
        edge_b: Beam | None = None,
        post: Beam | None = None,
        step_depth: float = 15.0,
        **kwargs,
    ) -> None:
        super().__init__(elements=(edge_a, edge_b, post), **kwargs)
        self.step_depth = step_depth

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

    def side_facing(self, beam: Beam, other: Beam) -> int:
        """The index of `beam`'s reference side that faces `other`."""
        incidence = beam_ref_side_incidence(other, beam, ignore_ends=True)
        return min(incidence, key=incidence.get)

    def side_against(self, beam: Beam, vector) -> int:
        """The index of `beam`'s reference side whose normal points most against `vector`."""
        angles = beam_ref_side_incidence_with_vector(beam, vector, ignore_ends=True)
        return max(angles, key=angles.get)

    def top_of(self, edge: Beam) -> Plane:
        """The upper face of `edge`: the one facing away from the post's foot."""
        down = self.post.centerline.midpoint - self.location
        face = edge.ref_sides[self.side_against(edge, down)]
        return Plane(face.point, face.normal)

    def add_extensions(self) -> None:
        """The post reaches above both edges; each edge reaches through the post, for its step to be cut from."""
        post_ends = [self.post.extension_to_plane(self.top_of(edge)) for edge in self.edges]
        self.post.add_blank_extension(max(s for s, _ in post_ends), max(e for _, e in post_ends), self.guid)
        for edge in self.edges:
            far_face = self.post.opp_side(self.side_facing(self.post, edge))
            edge.add_blank_extension(*edge.extension_to_plane(far_face), self.guid)

    def add_features(self) -> None:
        """Step each edge into its face of the post."""
        self.features = []
        for role, edge in zip(("edge_a", "edge_b"), self.edges):
            post_side = self.side_facing(self.post, edge)
            post_face = self.post.ref_sides[post_side]
            edge_side = self.side_against(edge, post_face.normal)
            _, edge_height = edge.get_dimensions_relative_to_side(edge_side)
            debug.show(post_face, f"post face for {role}")

            step = StepJoint.from_plane_and_beam(post_face, edge, step_depth=self.step_depth, ref_side_index=edge_side)
            notch = StepJointNotch.from_plane_and_beam(  # across the post's whole face
                edge.ref_sides[edge_side],
                self.post,
                step_depth=self.step_depth,
                strut_height=edge_height,
                ref_side_index=post_side,
            )
            edge.add_features(step)
            self.post.add_features(notch)
            self.features.extend([step, notch])
