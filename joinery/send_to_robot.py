# /// script
# requires-python = ">=3.9,<3.13"
# dependencies = ["compas_rrc==2.0.0"]
# ///
"""Send a program from `robot_execution.gh` to the robot over COMPAS RRC.

    uv run send_to_robot.py program.json                  # the robot, through the RRC driver on this machine
    uv run send_to_robot.py program.json --host 10.0.0.5  # the driver on another machine
    uv run send_to_robot.py program.json --dry-run        # print what would be sent, connect to nothing

The program (`MillingCell.program()` in robot_lib) is the bit's tip frames in the
robot's base frame, run in `wobj0` with the controller's active tool (t_RRC_Act),
which is never set here. Before any straight move the controller's TCP must be where
our tool's is, or it stops: with another tool active the bit would go elsewhere.
It runs outside Rhino: compas_rrc 2.0.0 asks for compas_fab 1.x. The Execute component of robot_execution.gh runs it too (sender_lib.py).

Moves are sent a few seconds ahead of the robot. End of input (Ctrl+D) stops the sending: the robot finishes what it has.
That is not an emergency stop, the pendant is.
"""

import argparse
import math
import os
import sys
import threading
import time
from collections import deque

import compas
import compas_rrc as rrc

JOINT_TOLERANCE = 0.5  # degrees, between the robot and the start of a program that starts where it stands
TCP_TOLERANCE = 1.0  # mm
TCP_ANGLE_TOLERANCE = 0.5  # degrees
AHEAD = 3.0  # seconds of moves sent ahead of the robot: enough that it never waits between short ones, few enough to stop soon
MIN_AHEAD = 3  # moves, however long they take: it needs the next ones to round a corner


def moves(program, speed):
    """The linear moves through every frame of `program`, each with the seconds it takes."""
    frames = program["frames"]
    for index, (before, frame) in enumerate(zip([program["start_tip"]] + frames, frames)):
        zone = rrc.Zone.FINE if index == len(frames) - 1 else rrc.Zone.Z1
        move = rrc.MoveToFrame(frame, speed, zone, rrc.Motion.LINEAR, feedback_level=rrc.FeedbackLevel.DONE)
        yield move, before.point.distance_to_point(frame.point) / speed


def finished(future, stopped):
    """Wait until the robot has done it: True, or False as soon as `stopped` is set.

    No time limit: it may be slowed or paused on the pendant. Nor does a stop wait for it: restarting the RRC program drops its moves, unanswered.
    """
    while not stopped.is_set():
        try:
            future.result(timeout=0.2)
            return True
        except rrc.TimeoutException:
            pass
    return False


def stream(abb, todo, stopped):
    """Send the moves `todo`, `AHEAD` seconds of them at a time, until they are done or `stopped` is set. Returns how many were sent, and how many of them are done."""
    todo = list(todo)
    ahead, sent, said = deque(), 0, 0.0

    def wait_for_oldest():
        """Until the robot has done it, or `stopped` is set."""
        nonlocal said
        if not finished(ahead[0][0], stopped):
            return
        ahead.popleft()
        if time.monotonic() - said > 1.0:
            said = time.monotonic()
            print(f"{sent - len(ahead)}/{len(todo)} moves")

    for move, seconds in todo:
        while len(ahead) >= MIN_AHEAD and sum(s for _, s in ahead) > AHEAD and not stopped.is_set():
            wait_for_oldest()
        if stopped.is_set():
            break
        ahead.append((abb.send(move), seconds))
        sent += 1
    while ahead and not stopped.is_set():
        wait_for_oldest()
    return sent, sent - len(ahead)


def watch_input(stopped):
    """Set `stopped` at the end of the input. Not sys.stdin: blocked on its lock, the interpreter can't exit."""
    while os.read(0, 1024):
        pass
    stopped.set()


def check_joints(abb, program):
    joints, _ = abb.send_and_wait(rrc.GetJoints(), timeout=10)
    off = max(abs(a - b) for a, b in zip(joints.values, program["start_joints"]))
    if not off <= JOINT_TOLERANCE:
        sys.exit(f"Stopped, nothing moved: the robot is {off:.1f} degrees from the program's start. Read the robot and write the program again.")


def check_tool(abb, program):
    tcp, tip = abb.send_and_wait(rrc.GetFrame(), timeout=10), program["start_tip"]
    distance = tcp.point.distance_to_point(tip.point)
    angle = max(math.degrees(getattr(tcp, a).angle(getattr(tip, a))) for a in ("xaxis", "yaxis", "zaxis"))
    print(f"The controller's TCP is {distance:.2f} mm and {angle:.2f} degrees from our tool's.")
    if not (distance <= TCP_TOLERANCE and angle <= TCP_ANGLE_TOLERANCE):  # also stops on nan
        sys.exit("Stopped, no straight move was sent: the active tool (t_RRC_Act) isn't our tool.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("program", help="the .json robot_execution.gh wrote")
    parser.add_argument("--speed", type=float, default=20.0, help="feed, mm/s (default 20)")
    parser.add_argument("--travel-speed", type=float, default=100.0, help="the joint move to the start, mm/s (default 100)")
    parser.add_argument("--host", default="localhost", help="where the RRC driver's rosbridge runs")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    program = compas.json_load(args.program)
    if "start_tip" not in program:
        sys.exit("This program holds flange frames, for tool0. Write it again: programs now hold the bit's tip.")
    from_here = program.get("from_here", False)
    to_start = rrc.MoveToJoints(program["start_joints"], [], args.travel_speed, rrc.Zone.FINE, feedback_level=rrc.FeedbackLevel.DONE)
    todo = list(moves(program, args.speed))
    start = "where the robot stands" if from_here else "a joint move"
    minutes = sum(seconds for _, seconds in todo) / 60
    print(f"{len(todo)} frames at {args.speed} mm/s, {minutes:.0f} min, starting from {start}, joints {[round(j, 1) for j in program['start_joints']]}")
    if args.dry_run:
        for instruction in ([] if from_here else [to_start]) + [move for move, _ in todo[:6]]:
            print(instruction.msg)
        return

    ros = rrc.RosClient(args.host, args.port)
    ros.run()
    abb = rrc.AbbClient(ros, "/rob1")
    try:
        abb.send_and_wait(rrc.SetWorkObject("wobj0"), timeout=10)
        if from_here:
            check_joints(abb, program)
            check_tool(abb, program)
        print("Ready. Area clear, spindle as it should be? Enter (go) to start, Ctrl+C (stop) to abort.")
        try:
            input()
        except EOFError:
            sys.exit("Aborted, nothing moved.")
        stopped = threading.Event()
        threading.Thread(target=watch_input, args=[stopped], daemon=True).start()
        if from_here:
            check_joints(abb, program)  # it may have been jogged while this waited
        else:
            print("Joint move to the start.")
            if not finished(abb.send(to_start), stopped):
                sys.exit("Stopped in the joint move to the start: no straight move was sent, the robot still finishes the joint move.")
            check_tool(abb, program)
        sent, done = stream(abb, todo, stopped)
        if done < len(todo):
            sys.exit(f"Stopped after {done} of {len(todo)} moves: the robot still finishes the {sent - done} it has. The bit may be in the wood.")
        print("Done.")
    except rrc.TimeoutException:
        sys.exit(
            "Stopped sending: the controller doesn't answer. Is it connected, and the RRC program running? "
            "It may still be finishing the moves it has, or the last run's, or be paused in them on the pendant."
        )
    except KeyboardInterrupt:
        # A Stop sent now would queue behind the moves already sent, so it's the pendant.
        print("Aborted here, but the controller still runs the moves it has. Stop it on the pendant.")
    finally:
        ros.close()
        ros.terminate()


if __name__ == "__main__":
    main()
