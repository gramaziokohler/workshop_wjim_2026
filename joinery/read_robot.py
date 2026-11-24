# /// script
# requires-python = ">=3.9,<3.13"
# dependencies = ["compas_rrc==2.0.0"]
# ///
"""Print where the robot is, read over COMPAS RRC, as JSON: its joints (degrees), and the TCP of its active tool (mm).

    uv run read_robot.py                  # the robot, through the RRC driver on this machine
    uv run read_robot.py --host 10.0.0.5  # the driver on another machine

It moves and sets nothing: GetJoints and GetFrame are all it sends. The TCP is that of t_RRC_Act, the tool
the last SetTool left there, in ob_RRC_Act, the work object the last SetWorkObject left there. Outside Rhino for the same reason as send_to_robot.py.
"""

import argparse
import json
import sys

import compas_rrc as rrc
from roslibpy.core import RosTimeoutError


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default="localhost", help="where the RRC driver's rosbridge runs")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--timeout", type=float, default=5.0, help="how long to wait for the controller, s")
    args = parser.parse_args()

    ros = rrc.RosClient(args.host, args.port)
    try:
        ros.run(timeout=5)
        abb = rrc.AbbClient(ros, "/rob1")
        joints, _ = abb.send_and_wait(rrc.GetJoints(), timeout=args.timeout)
        tcp = abb.send_and_wait(rrc.GetFrame(), timeout=args.timeout)
    except RosTimeoutError:
        sys.exit(f"No RRC driver at {args.host}:{args.port}: docker compose -f robot/docker-compose.yml up -d")
    except rrc.TimeoutException:
        sys.exit("The RRC driver is up, the controller doesn't answer. Is it connected, and the RRC program running?")
    finally:
        ros.close()
        ros.terminate()
    print(json.dumps({"joints": list(joints.values), "tcp": [list(v) for v in (tcp.point, tcp.xaxis, tcp.yaxis)]}))


if __name__ == "__main__":
    main()
