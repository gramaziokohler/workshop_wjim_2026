"""The milling cell: an ABB IRB2600 with the spindle, and a beam on a table.

Enough to answer "can the robot mill this?" before going to the robot:
place the beam, solve every toolpath frame, look at the robot along the way.
The cell is in millimetres; the robot, its tool and compas_fab are in metres
and are converted at the edges.

The robot is solved with compas_fab and its native (Rust) planner, on the
milling tool in `robot/milling_tool.json`: a toolpath frame is where the tip
of the bit goes, its Z into the wood.

Runs in Rhino 8 (Python 3.9) and in plain Python.
"""

from __future__ import annotations

import math
import os
import random
from functools import cache
from types import SimpleNamespace

import compas
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas.geometry import Vector
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotSemantics
from compas_fab.robots import TargetMode
from compas_fab_native import NativePlanner
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

ROBOT_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot")
URDF = os.path.join(ROBOT_FOLDER, "abb_irb2600_12_165.urdf")
SRDF = os.path.join(ROBOT_FOLDER, "abb_irb2600_12_165.srdf")
TOOL = os.path.join(ROBOT_FOLDER, "milling_tool.json")
HOME = [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]

# The cell stands next to the design (y = -3000) rather than inside it.
ROBOT_FRAME = Frame(Point(0, -3000, 0), Vector(1, 0, 0), Vector(0, 1, 0))
TABLE_FRAME = Frame(Point(1100, -3000, 400), Vector(1, 0, 0), Vector(0, 1, 0))

MM_PER_M = 1000.0


def _to_metres(frame):
    return Frame(frame.point.scaled(1 / MM_PER_M), frame.xaxis, frame.yaxis)


def _to_millimetres(frame):
    return Frame(frame.point.scaled(MM_PER_M), frame.xaxis, frame.yaxis)


def _joint_distance(a, b):
    return sum(abs(x - y) for x, y in zip(a, b))


def _load_robot_model():
    model = RobotModel.from_urdf_file(URDF)
    model.load_geometry(LocalPackageMeshLoader(ROBOT_FOLDER, "abb_irb2600_support"))
    return model


@cache
def _robot():
    """The robot model, the spindle attached to it, and the native planner; built once per session, it takes seconds."""
    model = _load_robot_model()
    tool = compas.json_load(TOOL)["tool"]
    cell = RobotCell(model, RobotSemantics.from_srdf_file(SRDF, model), tool_models={tool.name: tool})
    state = cell.default_cell_state()
    state.set_tool_attached_to_group(tool.name, cell.main_group_name)
    planner = NativePlanner()
    planner.set_robot_cell(cell, state)
    return SimpleNamespace(model=model, tool=tool, state=state, planner=planner)


@cache
def _drawn_robot():
    return _load_robot_model()


class MillingCell:
    """The robot, its spindle and the table the beam is clamped to.

    Parameters
    ----------
    robot_frame : Frame
        Where the robot's base sits.
    table_frame : Frame
        Where the beam's reference corner goes: the beam runs along its X, the
        machined side faces up along its Z.
    """

    def __init__(self, robot_frame=None, table_frame=None):
        self.robot_frame = robot_frame or ROBOT_FRAME.copy()
        self.table_frame = table_frame or TABLE_FRAME.copy()

    @classmethod
    def from_measured_points(cls, a, b, c, robot_frame=None):
        """The cell with its table where the robot measured it, three points in the robot's base frame (mm).

        `a` is the origin, `b` a point along X from it, `c` a point towards Y.
        """
        cell = cls(robot_frame)
        measured = Frame(a, Vector.from_start_end(a, b), Vector.from_start_end(a, c))
        cell.table_frame = cell.robot_frame.to_world_coordinates(measured)
        return cell

    @property
    def robot(self):
        """The robot model with its meshes, to draw. Its own copy: drawing scales it to millimetres in place."""
        return _drawn_robot()

    @property
    def tool(self):
        """The spindle (a compas_robots ToolModel, metres)."""
        return _robot().tool

    def placement(self, beam, ref_side_index):
        """The transformation that clamps `beam` with side `ref_side_index` up and that side's reference corner on the table frame."""
        return Transformation.from_frame_to_frame(beam.ref_sides[ref_side_index], self.table_frame)

    def setup(self, beam, toolpaths, side_index=None):
        """Clamp `beam` with side `side_index` up, that side's reference corner on the table frame.

        Without a `side_index`, the side the first milled toolpath comes from goes up.

        Returns
        -------
        tuple
            The beam's placement (a Transformation), the tool frames of this setup
            in the cell, and the toolpaths that need the beam turned to another side.
        """
        milled = [t for t in toolpaths if t.frames]
        if side_index is None:
            side_index = milled[0].ref_side_index if milled else 0

        on_this_side = [t for t in milled if t.ref_side_index == side_index]
        on_other_sides = [t for t in milled if t.ref_side_index != side_index]
        frames = [f for t in on_this_side for f in t.frames]
        placement = self.placement(beam, side_index)
        return placement, [f.transformed(placement) for f in frames], on_other_sides

    def flange_frame(self, tool_frame):
        """The flange (`tool0`) frame that puts the bit's tip on `tool_frame`."""
        tcp = Transformation.from_frame(_to_millimetres(self.tool.frame))
        return Frame.from_transformation(Transformation.from_frame(tool_frame) * tcp.inverse())

    def solve(self, tool_frames, start=None):
        """A joint configuration (radians) for every frame, each close to the previous one; None where out of reach.

        Solutions are collision-free between the robot and its spindle (compas_fab_native's sphere model).
        """
        configurations = []
        previous = start or HOME
        for frame in tool_frames:
            candidates = self._inverse_kinematics(frame)
            if not candidates:
                configurations.append(None)
                continue
            previous = min(candidates, key=lambda q: _joint_distance(q, previous))
            configurations.append(previous)
        return configurations

    def _inverse_kinematics(self, tool_frame):
        """Every joint configuration that puts the bit's tip on `tool_frame`."""
        robot = _robot()
        target = FrameTarget(
            _to_metres(self.robot_frame.to_local_coordinates(tool_frame)),
            TargetMode.TOOL,
        )
        try:
            return [q.joint_values for q in robot.planner.iter_inverse_kinematics(target, robot.state)]
        except InverseKinematicsError:
            return []

    def configuration(self, joint_values):
        """The compas_robots Configuration of `joint_values`."""
        return Configuration.from_revolute_values(joint_values, _robot().model.get_configurable_joint_names())

    def tool0(self, joint_values):
        """Where the flange is."""
        return self._forward_kinematics(joint_values, TargetMode.ROBOT)

    def tool_tip(self, joint_values):
        """Where the tip of the bit is."""
        return self._forward_kinematics(joint_values, TargetMode.TOOL)

    def _forward_kinematics(self, joint_values, target_mode):
        """By the same planner that solves the IK, in millimetres and in the cell."""
        robot = _robot()
        state = robot.state.copy()
        state.robot_configuration = self.configuration(joint_values)
        frame = robot.planner.forward_kinematics(state, target_mode)
        return self.robot_frame.to_world_coordinates(_to_millimetres(frame))

    def tool_mesh(self, joint_values):
        """The spindle's mesh where `joint_values` puts it, in millimetres and in the cell."""
        mesh = self.tool.root.visual[0].geometry.shape.meshes[0].scaled(MM_PER_M)
        return mesh.transformed(Transformation.from_frame(self.tool0(joint_values)))


def frame_along(frames, t):
    """The frame `t` of the way along the path through `frames` (0 = start, 1 = end), by distance travelled.

    Returns
    -------
    tuple
        The frame, with the orientation of the frame before it, and that frame's index.
    """
    lengths = [a.point.distance_to_point(b.point) for a, b in zip(frames, frames[1:])]
    remaining = min(max(t, 0.0), 1.0) * sum(lengths)
    for index, length in enumerate(lengths):
        if remaining <= length or index == len(lengths) - 1:
            share = min(remaining / length, 1.0) if length else 0.0
            start, end = frames[index], frames[index + 1]
            return Frame(start.point + (end.point - start.point) * share, start.xaxis, start.yaxis), index
        remaining -= length
    return frames[0], 0


if __name__ == "__main__":
    # Self-check: halfway along a path is halfway by distance, whatever the frames' spacing.
    path = [Frame(Point(x, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0)) for x in (0, 10, 100)]
    halfway, index = frame_along(path, 0.5)
    assert halfway.point.distance_to_point(Point(50, 0, 0)) < 1e-9 and index == 1
    assert frame_along(path, 0.0)[0].point == path[0].point and frame_along(path, 1.0)[0].point == path[-1].point

    # Self-check: the table frame from three measured points.
    a, b, c = Point(1100, 0, 400), Point(1200, 0, 400), Point(1100, 100, 400)
    measured = MillingCell.from_measured_points(a, b, c)
    assert measured.table_frame.point == TABLE_FRAME.point and measured.table_frame.xaxis == TABLE_FRAME.xaxis
    moved = MillingCell.from_measured_points(a, b, c, robot_frame=Frame(Point(10, 20, 0), Vector(0, 1, 0), Vector(-1, 0, 0)))
    assert moved.table_frame.point.distance_to_point(Point(10, 1120, 400)) < 1e-9
    sloppy = MillingCell.from_measured_points(a, b, Point(1105, 100, 401)).table_frame
    assert abs(sloppy.xaxis.dot(sloppy.yaxis)) < 1e-9 and sloppy.xaxis.dot(Vector(1, 0, 0)) > 1 - 1e-9

    # Self-check: the beam's reference corner for the side that goes up lands on the table frame.
    from compas.geometry import Line
    from compas_timber.elements import Beam

    beam = Beam.from_centerline(Line(Point(0, 0, 0), Point(1000, 0, 0)), width=80, height=160)
    for side_index in range(4):
        placement, _, _ = measured.setup(beam, [], side_index)
        corner = beam.ref_sides[side_index].transformed(placement)
        assert corner.point.distance_to_point(measured.table_frame.point) < 1e-6 and corner.normal.dot(measured.table_frame.normal) > 1 - 1e-9

    # Self-check: solving a tip frame the robot reaches gives a configuration that puts the tip there.
    cell = MillingCell()
    random.seed(0)
    solved = 0
    for _ in range(100):
        q = [random.uniform(-1.2, 1.2) for _ in range(6)]
        tip = cell.tool_tip(q)
        (solution,) = cell.solve([tip], start=q)
        if solution is None:
            continue  # q itself has the spindle in the robot's arm
        assert cell.tool_tip(solution).point.distance_to_point(tip.point) < 1e-3, (
            q,
            solution,
        )
        assert cell.flange_frame(tip).point.distance_to_point(cell.tool0(solution).point) < 1e-3, (q, solution)
        solved += 1
    assert solved > 50, solved
    print(f"IK puts the bit's tip on the target ({solved} of 100 random poses collision-free).")
