import json
import subprocess
import sys
import os

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from agy_bounded_allow import decide  # noqa: E402

WS = ["/home/athome/orca/workspaces/base_athome_ws/antigravity-home-40-mission-monitor"]


@pytest.mark.parametrize("cmd", [
    "git status", "git diff --stat", "git log --oneline -5", "git add tools/mission_monitor",
    "git commit -m 'feat: x'", "ls -la tools", "cat README.md | head -20", "grep -rn foo tools/",
    "cd tools/mission_monitor && env -u PYTHONPATH python3 -m pytest -q",
    "source /opt/ros/jazzy/setup.bash && colcon build --packages-select base_athome_skills",
    "ROS_DOMAIN_ID=42 python3 -m pytest -q test 2>&1", "mkdir -p tools/mission_monitor/static",
    "find . -name '*.py'", "git branch --show-current", "git worktree list", "xacro robot.urdf.xacro",
])
def test_allows_innocuous(cmd):
    assert decide(cmd, WS)["decision"] == "allow", cmd


@pytest.mark.parametrize("cmd", [
    "git push origin HEAD", "git reset --hard", "git clean -fdx", "git remote -v", "rm -rf build",
    "sudo apt install x", "ssh ubuntu@10.42.0.12", "curl http://x", "pip install flask",
    "ROS_DOMAIN_ID=10 ros2 topic pub /cmd_vel x", "ros2 topic pub /arm/joint_cmd x",
    "ros2 service call /arm/set_enable x", "echo x > file.py", "cat $(which python3)",
    "python3 server.py", "find . -delete", "git checkout main", "cd /etc && ls", "bash -c 'ls'",
    "systemctl --user restart realsense-nuc", "git status; rm -rf /",
])
def test_asks_for_everything_else(cmd):
    assert decide(cmd, WS)["decision"] == "ask", cmd


def test_outside_workspace_asks():
    assert decide("git status", ["/home/athome"])["decision"] == "ask"


def test_hook_protocol_roundtrip():
    payload = {"toolCall": {"name": "run_command", "args": {"CommandLine": "git status"}}, "workspacePaths": WS}
    out = subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), "agy_bounded_allow.py")],
                         input=json.dumps(payload), capture_output=True, text=True, timeout=10,
                         env={**os.environ, "HOME": "/tmp"})
    assert json.loads(out.stdout)["decision"] == "allow"
