"""send_to_robot.py beside Grasshopper: started, confirmed and stopped from a component, which never waits for it.

Two steps, as in a terminal: it connects and checks, and nothing moves before `go`. Its stdin is the
line to it: a newline is go, closing it is stop, and if Rhino dies it is closed too.
`python sender_lib.py` checks itself, without a robot.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import threading

FOLDER = os.path.dirname(os.path.abspath(__file__))


def send_command(program, speed, travel_speed):
    """The command line of send_to_robot.py, and the environment to run it in."""
    # Rhino started from the Dock has neither uv on its PATH nor a Python environment uv should inherit
    path = os.pathsep.join([os.environ.get("PATH", ""), os.path.expanduser("~/.local/bin"), "/opt/homebrew/bin"])
    env = {k: v for k, v in os.environ.items() if not k.startswith("PYTHON")}
    env.update(PATH=path, PYTHONUNBUFFERED="1")
    uv = shutil.which("uv", path=path)
    if uv is None:
        raise ValueError("uv not found: https://docs.astral.sh/uv")
    script = os.path.join(FOLDER, "send_to_robot.py")
    return [uv, "run", "--script", script, program, "--speed", str(speed), "--travel-speed", str(travel_speed)], env


class Sender:
    """A running send_to_robot.py. `on_line` is called from another thread for each line it prints, and when it ends."""

    def __init__(self, command, env=None, on_line=lambda: None):
        self.lines = []
        self.on_line = on_line
        self.process = subprocess.Popen(
            command, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1
        )
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.process.stdout:
            if line.strip():
                self.lines.append(line.rstrip())
                self.on_line()
        self.process.wait()
        self.on_line()

    @property
    def running(self):
        return self.process.poll() is None

    @property
    def ready(self):
        """Connected and checked, waiting for `go`. A `go` before that must not be kept for later."""
        return self.running and bool(self.lines) and self.lines[-1].startswith("Ready.")

    def go(self):
        if self.ready:
            self.lines.append("Go.")
            self.process.stdin.write("\n")
            self.process.stdin.flush()

    def stop(self):
        try:
            self.process.stdin.close()
        except OSError:
            pass  # it has ended already


if __name__ == "__main__":
    import sys
    import time
    from types import SimpleNamespace

    def wait_for(condition):
        end = time.time() + 10
        while not condition():
            assert time.time() < end, sender.lines
            time.sleep(0.02)

    # Self-check: go is refused until the sender is ready, and stop and go arrive.
    stand_in = [sys.executable, "-u", "-c", "import time; print('hi'); time.sleep(0.5); print('Ready. Go?'); input(); print('moving'); input()"]
    sender = Sender(stand_in)
    wait_for(lambda: sender.lines)
    sender.go()
    assert sender.lines == ["hi"], "go before ready is dropped"
    wait_for(lambda: sender.ready)
    sender.go()
    wait_for(lambda: "moving" in sender.lines)
    assert sender.running and not sender.ready
    sender.stop()
    wait_for(lambda: not sender.running)
    sender.stop()

    # Self-check: send_to_robot's stream, on a robot that isn't there.
    class Timeout(Exception):
        pass

    sys.modules["compas_rrc"] = SimpleNamespace(
        TimeoutException=Timeout, FeedbackLevel=SimpleNamespace(DONE=1), Zone=SimpleNamespace(FINE="fine", Z1="z1"), Motion=SimpleNamespace(LINEAR="L"),
        MoveToFrame=lambda frame, speed, zone, motion, feedback_level=0: SimpleNamespace(frame=frame, zone=zone, feedback_level=feedback_level),
    )
    from compas.geometry import Frame

    import send_to_robot

    class Robot:
        """Finishes a move each time something waits for the oldest one."""

        def __init__(self, stopped=None, stop_at=None):
            self.sent, self.done, self.most, self.stopped, self.stop_at = [], 0, 0, stopped, stop_at

        def send(self, instruction):
            assert instruction.feedback_level == 1
            self.sent.append(instruction)
            self.most = max(self.most, len(self.sent) - self.done)
            if len(self.sent) == self.stop_at:
                self.stopped.set()

            def result(timeout=None):
                self.done += 1

            return SimpleNamespace(result=result)

    def program(step, count=400):
        frames = [Frame([step * i, 0, 0], [1, 0, 0], [0, 1, 0]) for i in range(count)]
        return {"frames": frames, "start_tip": frames[0]}

    for step, ahead in ((1.0, 70), (500.0, 4)):  # mm a move, at 20 mm/s: short moves are sent well ahead, long ones aren't
        robot = Robot()
        assert send_to_robot.stream(robot, send_to_robot.moves(program(step), 20.0), threading.Event()) == 400
        assert [i.frame.point.x for i in robot.sent] == [step * i for i in range(400)] and robot.done == 400
        assert [i.zone for i in robot.sent] == ["z1"] * 399 + ["fine"]
        assert send_to_robot.MIN_AHEAD <= robot.most <= ahead, (step, robot.most)
    stopped = threading.Event()
    robot = Robot(stopped, stop_at=100)
    assert send_to_robot.stream(robot, send_to_robot.moves(program(1.0), 20.0), stopped) == 100 == len(robot.sent) == robot.done

    class Silent:  # a controller that never answers: stop is still heard
        def send(self, instruction):
            def result(timeout=None):
                stopped.set()
                raise Timeout()

            return SimpleNamespace(result=result)

    stopped, send_to_robot.STALL = threading.Event(), 0.1
    try:
        send_to_robot.stream(Silent(), send_to_robot.moves(program(1.0), 20.0), stopped)
        raise AssertionError("a robot that went silent was taken for done")
    except Timeout:
        pass
    print("The sender waits for go, stays a few moves ahead of the robot, and stops.")
