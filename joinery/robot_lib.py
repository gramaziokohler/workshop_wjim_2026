"""The milling cell: an ABB IRB2600 with a spindle, and a beam on a table.

Enough to answer "can the robot mill this?" before going to the robot:
place the beam, solve every toolpath frame, look at the robot along the way.
Everything is in millimetres; the robot model itself is in metres and is
converted at the edges.

Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import math
import os

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Rotation
from compas.geometry import Transformation
from compas.geometry import Vector
from compas.geometry import intersection_plane_circle
from compas.geometry import intersection_sphere_sphere

ROBOT_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot")
URDF = os.path.join(ROBOT_FOLDER, "abb_irb2600_12_165.urdf")

# The IRB2600-12/1.65 as four points in metres (zero position: upper arm up,
# forearm forward): axis 2, axis 3, wrist centre, flange. From the URDF.
IRB2600_POINTS = [Point(0.150, 0.0, 0.445), Point(0.150, 0.0, 1.145), Point(0.945, 0.0, 1.260), Point(1.030, 0.0, 1.260)]
JOINT_LIMITS = [(-3.14159, 3.14159), (-2.705, 1.658), (-2.705, 1.309), (-6.981, 6.981), (-2.094, 2.094), (-6.981, 6.981)]
HOME = [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]


class MillingCell(object):
    """The robot, its spindle and the table the beam is clamped to.

    Parameters
    ----------
    robot_frame : Frame
        Where the robot's base sits.
    table_frame : Frame
        The clamping position: the machined face of the beam is placed here, facing up (+Z).
    tool_length : float
        From the flange to the tip of the bit, along the spindle.
    """

    def __init__(self, robot_frame=None, table_frame=None, tool_length=250.0):
        # By default the cell stands next to the design (y = -3000) rather than inside it.
        self.robot_frame = robot_frame or Frame(Point(0, -3000, 0), Vector(1, 0, 0), Vector(0, 1, 0))
        self.table_frame = table_frame or Frame(Point(1100, -3000, 400), Vector(1, 0, 0), Vector(0, 1, 0))
        self.tool_length = tool_length
        self._robot = None

    @property
    def robot(self):
        """The robot model with its meshes, loaded once."""
        if self._robot is None:
            from compas_robots import RobotModel
            from compas_robots.resources import LocalPackageMeshLoader

            self._robot = RobotModel.from_urdf_file(URDF)
            self._robot.load_geometry(LocalPackageMeshLoader(ROBOT_FOLDER, "abb_irb2600_support"))
        return self._robot

    def placement(self, beam, ref_side_index, point):
        """The transformation that clamps `beam` with side `ref_side_index` up and `point` (on that side) at the table frame."""
        side = beam.ref_sides[ref_side_index]
        local = Frame(point, side.xaxis, side.yaxis)
        return Transformation.from_frame_to_frame(local, self.table_frame)

    def setup(self, beam, toolpaths):
        """Clamp `beam` for the side its first milled toolpath comes from.

        Returns
        -------
        tuple
            The beam's placement (a Transformation, or None if nothing is milled),
            the tool frames of this setup in the cell, and the toolpaths that need
            the beam turned to another side.
        """
        milled = [t for t in toolpaths if t.frames]
        if not milled:
            return None, [], []
        side = milled[0].ref_side_index
        here = [t for t in milled if t.ref_side_index == side]
        frames = [f for t in here for f in t.frames]
        # centre the work on the table: the middle of the frames, on the machined face
        middle = Point(*[sum(c) / len(frames) for c in zip(*[f.point for f in frames])])
        x, y, _ = beam.ref_sides[side].to_local_coordinates(middle)
        placement = self.placement(beam, side, beam.ref_sides[side].to_world_coordinates(Point(x, y, 0)))
        return placement, [f.transformed(placement) for f in frames], [t for t in milled if t.ref_side_index != side]

    def flange_frame(self, tool_frame):
        """The flange frame that puts the bit's tip on `tool_frame` (its Z pointing into the material)."""
        return Frame(tool_frame.point - tool_frame.zaxis * self.tool_length, tool_frame.xaxis, tool_frame.yaxis)

    def solve(self, tool_frames, start=None):
        """A joint configuration (radians) for every frame, each close to the previous one; None where out of reach."""
        configurations, previous = [], start or HOME
        for frame in tool_frames:
            flange = self.flange_frame(frame)
            local = self.robot_frame.to_local_coordinates(flange)
            local_m = Frame(Point(*[c / 1000.0 for c in local.point]), local.xaxis, local.yaxis)
            candidates = [q for q in irb2600_inverse(local_m) if _within_limits(q)]
            if not candidates:
                configurations.append(None)
                continue
            previous = min(candidates, key=lambda q: sum(abs(a - b) for a, b in zip(q, previous)))
            configurations.append(previous)
        return configurations

    def configuration(self, joint_values):
        from compas_robots import Configuration

        return Configuration.from_revolute_values(joint_values, self.robot.get_configurable_joint_names())

    def tool0(self, joint_values):
        """Forward kinematics of the URDF, in millimetres and in the cell: where the flange is."""
        frame = self.robot.forward_kinematics(self.configuration(joint_values), link_name="tool0")
        frame = Frame(Point(*[c * 1000.0 for c in frame.point]), frame.xaxis, frame.yaxis)
        return self.robot_frame.to_world_coordinates(frame)


def _within_limits(q):
    return all(lo <= a <= hi for a, (lo, hi) in zip(q, JOINT_LIMITS))


# ---------------------------------------------------------------------------
# Inverse kinematics for a spherical-wrist robot.
#
# Vendored from compas_fab 2.1.0 (MIT licence), backends/kinematics/solvers:
# `inverse_kinematics_spherical_wrist` and the ABB joint conventions of
# `ABB_IRB4600_40_255Kinematics`, with the IRB2600's own points. compas_fab
# itself is not used because importing it pulls in its ROS client.
# ---------------------------------------------------------------------------


def irb2600_inverse(flange_frame):
    """The (up to 8) joint configurations that put the IRB2600's flange (`tool0`) on `flange_frame`, in metres."""
    solutions = []
    for q1, q2, q3, q4, q5, q6 in _inverse_kinematics_spherical_wrist(flange_frame, IRB2600_POINTS):
        q = [-q1, q2 + math.pi / 2, q3 - math.pi / 2, -q4, q5, -q6 + math.pi / 2]
        solutions.append([_wrap(a) for a in q])
    return solutions


def _wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# compas' Frame.to_local_coordinates and Frame.transform invert and decompose 4x4
# matrices, which made the solve take ~6 ms per frame. The frames here are
# orthonormal, so dot products and a turn about the frame's own Z do the same.


def _local(frame, point):
    v = point - frame.point
    return v.dot(frame.xaxis), v.dot(frame.yaxis), v.dot(frame.zaxis)


def _world(frame, x, y, z):
    return frame.point + frame.xaxis * x + frame.yaxis * y + frame.zaxis * z


def _spun(frame, angle):
    """`frame` turned by `angle` about its own Z (only its axes are used after)."""
    c, s = math.cos(angle), math.sin(angle)
    return Frame(frame.point, frame.xaxis * c + frame.yaxis * s, frame.yaxis * c - frame.xaxis * s)


def _inverse_kinematics_spherical_wrist(target_frame, points):
    p1, p2, p3, p4 = points

    end_frame = Frame(target_frame.point, target_frame.yaxis, target_frame.xaxis)

    wrist_offset = p4.x - p3.x
    lower_arm_length = p2.z - p1.z
    upper_arm_length = (p3 - p2).length
    axis4_offset_angle = math.atan2(p3.z - p2.z, p3.x - p2.x)
    wrist = _world(end_frame, 0, 0, wrist_offset)

    p1_proj = p1.copy()
    p1_proj.y = p2.y

    front = -1 * math.atan2(wrist.y, wrist.x)
    solutions = []
    for axis1_angle in (front, _wrap(front + math.pi)):
        rot1 = Rotation.from_axis_and_angle([0, 0, 1], -1 * axis1_angle, point=[0, 0, 0])
        p1a = p1_proj.transformed(rot1)
        elbow_dir = Vector(1, 0, 0).transformed(rot1)
        elbow_frame = Frame(p1a, elbow_dir, [0, 0, 1])
        elbow_plane = (p1a, elbow_frame.normal)

        result = intersection_sphere_sphere((p1a, lower_arm_length), (wrist, upper_arm_length))
        if not result or result[0] != "circle":
            continue  # out of reach on this side
        _, (center, radius, normal) = result
        intersections = intersection_plane_circle(elbow_plane, ((center, normal), radius))
        if not intersections:
            continue

        for elbow_pt in intersections:
            elbow_pt = Point(*elbow_pt)
            elbowx, elbowy, _ = _local(elbow_frame, elbow_pt)
            wristx, wristy, _ = _local(elbow_frame, wrist)

            axis2_angle = math.atan2(elbowy, elbowx)
            axis3_angle = math.pi - axis2_angle + math.atan2(wristy - elbowy, wristx - elbowx) - axis4_offset_angle

            temp_frame = _spun(elbow_frame, axis2_angle + axis3_angle)
            axis4_frame = Frame(wrist, temp_frame.zaxis, temp_frame.yaxis * -1.0)

            for flip in (False, True):
                axis6x, axis6y, _ = _local(axis4_frame, end_frame.point)
                axis4_angle = math.atan2(axis6y, axis6x)
                if flip:
                    axis4_angle = _wrap(axis4_angle + math.pi)

                axis5_frame = _spun(axis4_frame, axis4_angle)
                axis5_frame = Frame(wrist, axis5_frame.zaxis * -1, axis5_frame.xaxis)
                axis6x, axis6y, _ = _local(axis5_frame, end_frame.point)
                axis5_angle = math.atan2(axis6y, axis6x)

                axis6_frame = _spun(axis5_frame, axis5_angle)
                axis6_frame = Frame(wrist, axis6_frame.yaxis * -1, axis6_frame.zaxis)
                endx, endy, _ = _local(axis6_frame, _world(end_frame, 1, 0, 0))

                solutions.append(
                    [
                        axis1_angle,
                        -axis2_angle,
                        _wrap(-axis3_angle + math.pi),
                        _wrap(axis4_angle + math.pi / 2),
                        axis5_angle,
                        math.atan2(endy, endx),
                    ]
                )
    return solutions


if __name__ == "__main__":
    # Self-check: the IK must reproduce the URDF's own forward kinematics.
    import random

    cell = MillingCell()
    random.seed(0)
    for _ in range(100):
        q = [random.uniform(lo * 0.8, hi * 0.8) for lo, hi in JOINT_LIMITS]
        target = cell.tool0(q)
        tip = Frame(target.point + target.zaxis * cell.tool_length, target.xaxis, target.yaxis)
        (solution,) = cell.solve([tip], start=q)
        assert solution is not None, q
        assert cell.tool0(solution).point.distance_to_point(target.point) < 1e-3, (q, solution)
    print("IK reproduces forward kinematics.")
