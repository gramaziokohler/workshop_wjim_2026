# /// script
# requires-python = ">=3.9,<3.13"
# dependencies = ["compas_rrc==2.0.0"]
# ///
"""Send a program from `robot_test.gh` to the robot over COMPAS RRC.

    uv run send_to_robot.py program.json                  # the robot, through the RRC driver on this machine
    uv run send_to_robot.py program.json --host 10.0.0.5  # the driver on another machine
    uv run send_to_robot.py program.json --dry-run        # print what would be sent, connect to nothing

The program (`MillingCell.program()` in robot_lib) is the bit's tip frames in the
robot's base frame, run in `wobj0` with the controller's active tool (t_RRC_Act),
which is never set here. Before any straight move the controller's TCP must be where
our tool's is, or it stops: with another tool active the bit would go elsewhere.
It runs outside Rhino: compas_rrc 2.0.0 asks for compas_fab 1.x.
"""

import argparse
import math
import sys

import compas
import compas_rrc as rrc

JOINT_TOLERANCE = 0.5  # degrees, between the robot and the start of a program that starts where it stands
TCP_TOLERANCE = 1.0  # mm
TCP_ANGLE_TOLERANCE = 0.5  # degrees


def moves(program, speed):
    """The linear moves through every frame of `program`."""
    frames = program["frames"]
    for index, frame in enumerate(frames):
        zone = rrc.Zone.FINE if index == len(frames) - 1 else rrc.Zone.Z1
        yield rrc.MoveToFrame(frame, speed, zone, rrc.Motion.LINEAR)


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
    parser.add_argument("program", help="the .json robot_test.gh wrote")
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
    to_start = rrc.MoveToJoints(program["start_joints"], [], args.travel_speed, rrc.Zone.FINE)
    todo = list(moves(program, args.speed))
    start = "where the robot stands" if from_here else "a joint move"
    print(f"{len(todo)} frames at {args.speed} mm/s, starting from {start}, joints {[round(j, 1) for j in program['start_joints']]}")
    if args.dry_run:
        for instruction in ([] if from_here else [to_start]) + todo[:6]:
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
        input("Connected. Area clear, spindle as it should be? Enter to start, Ctrl+C to abort. ")
        if from_here:
            check_joints(abb, program)  # it may have been jogged while this waited
        else:
            abb.send_and_wait(to_start)
            check_tool(abb, program)
        for instruction in todo[:-1]:
            abb.send(instruction)
        abb.send_and_wait(todo[-1])  # no timeout: it returns when the last move is done
        print("Done.")
    except rrc.TimeoutException:
        sys.exit("Stopped: the controller doesn't answer. Is it connected, and the RRC program running?")
    except KeyboardInterrupt:
        # A Stop sent now would queue behind the moves already sent, so it's the pendant.
        print("Aborted here, but the controller still runs what it was sent. Stop it on the pendant.")
    finally:
        ros.close()
        ros.terminate()


if __name__ == "__main__":
    main()
