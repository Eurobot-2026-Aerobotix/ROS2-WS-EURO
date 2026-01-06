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

    # Workspace paths
    agent_cmd = "ros2 run micro_ros_agent micro_ros_agent"

    # STM32 agent command (using sg dialout for serial port access without sudo)
    # Need to source ROS setup because sg creates a new shell
    stm32_cmd = f"sg dialout -c 'source ~/.bashrc && {agent_cmd} serial --dev /dev/ttyACM0 -b 115200'"

    # tmux session name
    session_name = "micro_ros"

    # Kill old session
    run_cmd(["tmux", "kill-session", "-t", session_name])

    # Create a new session (detached) - need to wrap command in bash -c for tmux
    subprocess.call([
        "tmux", "new-session", "-d", "-s", session_name, 
        "bash", "-c", stm32_cmd
    ])

    # Optional: set layout
    subprocess.call([
        "tmux", "select-layout", "-t", session_name, "even-vertical"
    ])

    # Attach to the session
    subprocess.call([
        "tmux", "attach-session", "-t", session_name
    ])

    print("✅ micro-ROS agent started in tmux session:", session_name)

if __name__ == "__main__":
    main()