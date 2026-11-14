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

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas.geometry import Vector

ROBOT_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot")
URDF = os.path.join(ROBOT_FOLDER, "abb_irb2600_12_165.urdf")
SRDF = os.path.join(ROBOT_FOLDER, "abb_irb2600_12_165.srdf")
TOOL = os.path.join(ROBOT_FOLDER, "milling_tool.json")
HOME = [0.0, 0.0, 0.0, 0.0, math.pi / 2, 0.0]

_cell = {}  # the robot cell and its planner: built once per Python session, they take seconds


def _robot_cell():
    """The compas_fab robot cell with the spindle attached, its state, and the native planner."""
    if not _cell:
        import compas
        from compas_fab.robots import RobotCell
        from compas_fab.robots import RobotSemantics
        from compas_fab_native import NativePlanner
        from compas_robots import RobotModel
        from compas_robots.resources import LocalPackageMeshLoader

        model = RobotModel.from_urdf_file(URDF)
        model.load_geometry(LocalPackageMeshLoader(ROBOT_FOLDER, "abb_irb2600_support"))
        tool = compas.json_load(TOOL)["tool"]
        cell = RobotCell(model, RobotSemantics.from_srdf_file(SRDF, model), tool_models={tool.name: tool})
        state = cell.default_cell_state()
        state.set_tool_attached_to_group(tool.name, cell.main_group_name)
        planner = NativePlanner()
        planner.set_robot_cell(cell, state)
        _cell.update(model=model, tool=tool, cell=cell, state=state, planner=planner)
    return _cell


class MillingCell(object):
    """The robot, its spindle and the table the beam is clamped to.

    Parameters
    ----------
    robot_frame : Frame
        Where the robot's base sits.
    table_frame : Frame
        The clamping position: the machined face of the beam is placed here, facing up (+Z).
    """

    def __init__(self, robot_frame=None, table_frame=None):
        # By default the cell stands next to the design (y = -3000) rather than inside it.
        self.robot_frame = robot_frame or Frame(Point(0, -3000, 0), Vector(1, 0, 0), Vector(0, 1, 0))
        self.table_frame = table_frame or Frame(Point(1100, -3000, 400), Vector(1, 0, 0), Vector(0, 1, 0))

    @property
    def robot(self):
        """The robot model with its meshes, to draw. Its own copy: drawing scales it to millimetres in place."""
        c = _robot_cell()
        if "drawn" not in c:
            from compas_robots import RobotModel
            from compas_robots.resources import LocalPackageMeshLoader

            c["drawn"] = RobotModel.from_urdf_file(URDF)
            c["drawn"].load_geometry(LocalPackageMeshLoader(ROBOT_FOLDER, "abb_irb2600_support"))
        return c["drawn"]

    @property
    def tool(self):
        """The spindle (a compas_robots ToolModel, metres)."""
        return _robot_cell()["tool"]

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
        """The flange (`tool0`) frame that puts the bit's tip on `tool_frame`."""
        tcp = self.tool.frame
        tcp_mm = Frame(Point(*[c * 1000.0 for c in tcp.point]), tcp.xaxis, tcp.yaxis)
        flange = Transformation.from_frame(tool_frame) * Transformation.from_frame(tcp_mm).inverse()
        return Frame.from_transformation(flange)

    def solve(self, tool_frames, start=None):
        """A joint configuration (radians) for every frame, each close to the previous one; None where out of reach.

        Solutions are collision-free between the robot and its spindle (compas_fab_native's sphere model).
        """
        from compas_fab.backends.exceptions import InverseKinematicsError
        from compas_fab.robots import FrameTarget
        from compas_fab.robots import TargetMode

        c = _robot_cell()
        configurations, previous = [], start or HOME
        for frame in tool_frames:
            local = self.robot_frame.to_local_coordinates(frame)
            target = FrameTarget(Frame(Point(*[v / 1000.0 for v in local.point]), local.xaxis, local.yaxis), TargetMode.TOOL)
            try:
                candidates = [q.joint_values for q in c["planner"].iter_inverse_kinematics(target, c["state"])]
            except InverseKinematicsError:
                configurations.append(None)
                continue
            previous = min(candidates, key=lambda q: sum(abs(a - b) for a, b in zip(q, previous)))
            configurations.append(previous)
        return configurations

    def configuration(self, joint_values):
        from compas_robots import Configuration

        return Configuration.from_revolute_values(joint_values, _robot_cell()["model"].get_configurable_joint_names())

    def _forward(self, joint_values, target_mode):
        """Forward kinematics by the planner that solves the IK, in millimetres and in the cell."""
        c = _robot_cell()
        state = c["state"].copy()
        state.robot_configuration = self.configuration(joint_values)
        frame = c["planner"].forward_kinematics(state, target_mode)
        frame = Frame(Point(*[v * 1000.0 for v in frame.point]), frame.xaxis, frame.yaxis)
        return self.robot_frame.to_world_coordinates(frame)

    def tool0(self, joint_values):
        """Where the flange is."""
        from compas_fab.robots import TargetMode

        return self._forward(joint_values, TargetMode.ROBOT)

    def tool_tip(self, joint_values):
        """Where the tip of the bit is."""
        from compas_fab.robots import TargetMode

        return self._forward(joint_values, TargetMode.TOOL)

    def tool_mesh(self, joint_values):
        """The spindle's mesh where `joint_values` puts it, in millimetres and in the cell."""
        mesh = self.tool.root.visual[0].geometry.shape.meshes[0].scaled(1000.0)
        return mesh.transformed(Transformation.from_frame(self.tool0(joint_values)))


if __name__ == "__main__":
    # Self-check: solving a tip frame the robot reaches gives a configuration that puts the tip there.
    import random

    cell = MillingCell()
    random.seed(0)
    solved = 0
    for _ in range(100):
        q = [random.uniform(-1.2, 1.2) for _ in range(6)]
        tip = cell.tool_tip(q)
        (solution,) = cell.solve([tip], start=q)
        if solution is None:
            continue  # q itself has the spindle in the robot's arm
        assert cell.tool_tip(solution).point.distance_to_point(tip.point) < 1e-3, (q, solution)
        assert cell.flange_frame(tip).point.distance_to_point(cell.tool0(solution).point) < 1e-3, (q, solution)
        solved += 1
    assert solved > 50, solved
    print("IK puts the bit's tip on the target ({} of 100 random poses collision-free).".format(solved))
