"""HelloButtJoint: the "hello world" of the joint editor.

Two beams, T topology: `main_beam` ends against the side of `cross_beam`,
and is cut flush to it. Nothing else.

Uncomment the steps in `add_features` one at a time:

1. Pick the face by hand: cut `main_beam` on cross side 3. Works, for these beams.
   Now, in `hello_butt.fixture.json`, flip the main beam to come from the other side
   (y: -550 -> 550). Side 3 is now the far face, and the cut takes the whole main
   beam away: the joint was *modelled* for one situation.
2. Look at the topology instead: the angle between each side of the cross beam
   and the main beam's centerline, drawn on each side.
3. Let the joint pick the face: the one whose angle is smallest. Flip the beam
   again: it follows. Now the joint is *parametric*.
"""

from __future__ import annotations

from compas.geometry import Plane
from compas_timber.connections import Joint
from compas_timber.connections import JointTopology
from compas_timber.elements import Beam
from compas_timber.fabrication import JackRafterCut

# STEP 2
# import math
# from compas_timber.connections.utilities import beam_ref_side_incidence

import debug


class HelloButtJoint(Joint):
    """`main_beam` ends against `cross_beam` and is cut flush to its side."""

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_T
    MIN_ELEMENT_COUNT = 2
    MAX_ELEMENT_COUNT = 2

    def __init__(self, main_beam: Beam | None = None, cross_beam: Beam | None = None, **kwargs) -> None:
        super().__init__(elements=(main_beam, cross_beam), **kwargs)

    @property
    def main_beam(self) -> Beam:
        return self.elements[0]

    @property
    def cross_beam(self) -> Beam:
        return self.elements[1]

    def add_features(self) -> None:
        self.features = []

        # STEP 1: we pick the side, by looking at the viewport.
        cross_side = 3

        # STEP 2: how does each side of the cross beam sit relative to the main beam?
        # incidence = beam_ref_side_incidence(self.main_beam, self.cross_beam)
        # for side, angle in incidence.items():
        #     debug.show(self.cross_beam.ref_sides[side], f"side {side}: {math.degrees(angle):.0f} deg")

        # STEP 3: the side facing the main beam is the one with the smallest angle.
        # cross_side = min(incidence, key=incidence.get)

        face = self.cross_beam.ref_sides[cross_side]
        debug.show(face, f"cut on cross side {cross_side}")

        # the side's normal points out of the cross beam; the cut keeps what is behind the plane
        cut_plane = Plane(face.point, -face.normal)
        cut = JackRafterCut.from_plane_and_beam(cut_plane, self.main_beam)
        self.main_beam.add_features(cut)
        self.features.append(cut)
