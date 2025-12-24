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
    ros2_ws = "/home/ubuntu/uros_ws"
    setup_script = os.path.join(ros2_ws, "install/setup.bash")
    agent_cmd = "ros2 run micro_ros_agent micro_ros_agent"

    # STM32 agent command
    stm32_cmd = f"bash -c 'source {setup_script} && {agent_cmd} serial --dev /dev/ttyACM0 -b 115200'"

    # tmux session name
    session_name = "micro_ros"

    # Kill old session
    run_cmd(["tmux", "kill-session", "-t", session_name])

    # Create a new session (detached)
    subprocess.call([
        "tmux", "new-session", "-d", "-s", session_name, stm32_cmd
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