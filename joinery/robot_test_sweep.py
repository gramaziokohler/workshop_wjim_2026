"""A slow, short move to try the real robot with, for the Sweep component of robot_execution.gh.

From the pose the robot is in: straight up, then to the right (seen from behind the robot) at that height.
The tool keeps its orientation. `python robot_test_sweep.py` checks itself.
"""

import math

from compas.geometry import Frame
from robot_lib import MM_PER_M
from robot_lib import MillingCell
from robot_lib import _robot

STEP = 5.0  # mm between the poses that are checked
MAX_JOINT_STEP = 3.0  # degrees between two of those: more is a singularity or a flip
LIMIT_MARGIN = 10.0  # degrees
WRIST_MARGIN = 15.0  # degrees axis 5 stays from 0, where the wrist is singular
MIN_CLEARANCE = 10.0  # mm between the arm and the spindle (the native planner's sphere model)


def sweep(cell, joints, lift, length):
    """The tip frames of the move from `joints` (radians): the start, the top of the lift, the end."""
    start = cell.tool_tip(joints)
    up, right = cell.robot_frame.zaxis * lift, cell.robot_frame.yaxis * -length
    return [Frame(p, start.xaxis, start.yaxis) for p in (start.point, start.point + up, start.point + up + right)]


def check(cell, joints, corners):
    """The joints (radians) every `STEP` mm along `corners`, the first being `joints`. Raises if any isn't safe to run."""
    frames = []
    for a, b in zip(corners, corners[1:]):
        n = max(1, math.ceil(a.point.distance_to_point(b.point) / STEP))
        frames += [Frame(a.point + (b.point - a.point) * (i / n), a.xaxis, a.yaxis) for i in range(1, n + 1)]
    path = [list(joints)] + cell.solve(frames, start=joints)
    if None in path:
        raise ValueError("{} of {} poses are out of reach, or the spindle hits the arm.".format(path.count(None), len(path)))
    limits = [(j.limit.lower, j.limit.upper) for j in _robot().model.get_configurable_joints()]
    for a, b in zip(path, path[1:]):
        step = max(abs(math.degrees(x - y)) for x, y in zip(a, b))
        if step > MAX_JOINT_STEP:
            raise ValueError("A joint turns {:.1f} degrees in {} mm: too close to a singularity.".format(step, STEP))
    for q in path:
        if min(math.degrees(min(v - low, high - v)) for v, (low, high) in zip(q, limits)) < LIMIT_MARGIN:
            raise ValueError("Within {} degrees of a joint limit.".format(LIMIT_MARGIN))
        if abs(math.degrees(q[4])) < WRIST_MARGIN:
            raise ValueError("Axis 5 is within {} degrees of 0.".format(WRIST_MARGIN))
    clearance = min(_robot().planner.client.core.clearance(path)) * MM_PER_M
    if clearance < MIN_CLEARANCE:
        raise ValueError("The spindle comes within {:.0f} mm of the arm.".format(clearance))
    return path, clearance


def program(cell, joints, corners):
    """What send_to_robot.py sends. `from_here`: the robot must stand at `joints` (radians), there is no joint move to them."""
    ends = [b for a, b in zip(corners, corners[1:]) if a.point.distance_to_point(b.point) > 1e-6]
    return dict(cell.program(ends, [joints] * len(ends)), from_here=True)


if __name__ == "__main__":
    # Self-check: from a pose the real robot was in, up and then right, never lower, in one arm configuration.
    cell = MillingCell()
    joints = [math.radians(v) for v in (0.44, 14.43, 16.96, -73.52, -39.59, 8.16)]
    corners = sweep(cell, joints, 50.0, 300.0)
    path, clearance = check(cell, joints, corners)
    tips = [cell.robot_frame.to_local_coordinates(cell.tool_tip(q).point) for q in path]
    assert abs(tips[-1].z - tips[0].z - 50) < 1e-3 and abs(tips[-1].y - tips[0].y + 300) < 1e-3 and abs(tips[-1].x - tips[0].x) < 1e-3
    assert all(b.z > a.z - 1e-3 and b.y < a.y + 1e-3 for a, b in zip(tips, tips[1:]))
    todo = program(cell, joints, corners)
    assert len(todo["frames"]) == 2 and all(abs(math.radians(a) - b) < 1e-9 for a, b in zip(todo["start_joints"], joints))
    assert todo["frames"][-1].point.distance_to_point(cell.robot_frame.to_local_coordinates(cell.tool_tip(path[-1]).point)) < 1e-3
    assert todo["start_tip"].point.distance_to_point(cell.robot_frame.to_local_coordinates(corners[0].point)) < 1e-3 and todo["from_here"]
    # Self-check: a pose at the wrist's singularity, and a move out of reach, are refused.
    for bad, length in (((0, 14, 17, -73, -5, 8), 300.0), ((0.44, 14.43, 16.96, -73.52, -39.59, 8.16), 3000.0)):
        bad = [math.radians(v) for v in bad]
        try:
            check(cell, bad, sweep(cell, bad, 50.0, length))
        except ValueError:
            continue
        raise AssertionError("not refused: {}, {}".format(bad, length))
    print("The sweep goes up and right, and refuses what it can't do safely ({} poses, spindle {:.0f} mm from the arm).".format(len(path), clearance))
