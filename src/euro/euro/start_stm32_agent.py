#!/usr/bin/env python3
import subprocess
import os
import sys

def run_cmd(cmd):
    """Run a shell command safely."""
    return subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    # Check if tmux is installed
    if run_cmd(['which', 'tmux']) != 0:
        print("❌ tmux is not installed. Install it with:")
        print("   sudo apt install tmux")
        sys.exit(1)

    # Bashrc path
    bashrc = os.path.expanduser("~/.bashrc")
    agent_cmd = "ros2 run micro_ros_agent micro_ros_agent"

    # ESP32 agent commands for 3 agents - source bashrc (which already has ROS2 sourced)
    stm32_cmd = f"bash -c 'source {bashrc} && {agent_cmd} serial --dev /dev/ttyACM0 -b 115200'"
   

    # tmux session name
    session_name = "stm32_agents"

    # Kill old session
    run_cmd(["tmux", "kill-session", "-t", session_name])

    # Create a new session with first agent (detached)
    subprocess.call([
        "tmux", "new-session", "-d", "-s", session_name, stm32_cmd
    ])

    # Set even horizontal layout
    subprocess.call([
        "tmux", "select-layout", "-t", session_name, "even-horizontal"
    ])

    print("✅ STM32 micro-ROS agents starting in tmux session:", session_name)
    print()
    print("To attach to the session, run:")
    print(f"   tmux attach -t {session_name}")
    print()
    print("To detach: Press Ctrl+B, then D")
    print("To kill session: tmux kill-session -t", session_name)

    # Attach to the session
    subprocess.call([
        "tmux", "attach-session", "-t", session_name
    ])

if __name__ == "__main__":
    main()
