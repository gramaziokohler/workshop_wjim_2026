# /// script
# requires-python = ">=3.9,<3.13"
# dependencies = ["compas_rrc==2.0.0"]
# ///
"""Send a program from `joinery.gh` to the robot over COMPAS RRC.

    uv run send_to_robot.py program.json                  # the robot, through the RRC driver on this machine
    uv run send_to_robot.py program.json --host 10.0.0.5  # the driver on another machine
    uv run send_to_robot.py program.json --dry-run        # print what would be sent, connect to nothing

The program (`MillingCell.program()` in robot_lib) is flange frames in the
robot's base frame, so the controller runs `tool0` in `wobj0` and its own tool
data plays no part. It runs outside Rhino: compas_rrc 2.0.0 asks for compas_fab 1.x.
"""

import argparse

import compas
import compas_rrc as rrc


def instructions(program, speed, travel_speed):
    """The RRC instructions for `program`: a joint move to the solved start, then linear moves through every frame."""
    frames = program["frames"]
    yield rrc.SetTool("tool0")
    yield rrc.SetWorkObject("wobj0")
    yield rrc.MoveToJoints(program["start_joints"], [], travel_speed, rrc.Zone.FINE)
    for index, frame in enumerate(frames):
        zone = rrc.Zone.FINE if index == len(frames) - 1 else rrc.Zone.Z1
        yield rrc.MoveToFrame(frame, speed, zone, rrc.Motion.LINEAR)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("program", help="the .json joinery.gh wrote")
    parser.add_argument("--speed", type=float, default=20.0, help="feed, mm/s (default 20)")
    parser.add_argument("--travel-speed", type=float, default=100.0, help="the joint move to the start, mm/s (default 100)")
    parser.add_argument("--host", default="localhost", help="where the RRC driver's rosbridge runs")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    program = compas.json_load(args.program)
    todo = list(instructions(program, args.speed, args.travel_speed))
    print(f"{len(program['frames'])} frames at {args.speed} mm/s, starting from joints {[round(j, 1) for j in program['start_joints']]}")
    if args.dry_run:
        for instruction in todo[:6]:
            print(instruction.msg)
        return

    ros = rrc.RosClient(args.host, args.port)
    ros.run()
    abb = rrc.AbbClient(ros, "/rob1")
    try:
        input("Connected. Spindle running, area clear? Enter to start, Ctrl+C to abort. ")
        for instruction in todo[:-1]:
            abb.send(instruction)
        abb.send_and_wait(todo[-1])  # no timeout: it returns when the last move is done
        print("Done.")
    except KeyboardInterrupt:
        # A Stop sent now would queue behind the moves already sent, so it's the pendant.
        print("Aborted here, but the controller still runs what it was sent. Stop it on the pendant.")
    finally:
        ros.close()
        ros.terminate()


if __name__ == "__main__":
    main()
