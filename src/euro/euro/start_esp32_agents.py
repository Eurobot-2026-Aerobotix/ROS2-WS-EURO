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

    # ESP32 agent commands for 3 agents
    esp1_cmd = f"bash -c 'source {setup_script} && {agent_cmd} serial --dev /dev/esp1 -b 115200 '"
    esp2_cmd = f"bash -c 'source {setup_script} && {agent_cmd} serial --dev /dev/esp2 -b 115200 '"
    esp3_cmd = f"bash -c 'source {setup_script} && {agent_cmd} serial --dev /dev/esp3 -b 115200 '"

    # tmux session name
    session_name = "esp32_agents"

    # Kill old session
    run_cmd(["tmux", "kill-session", "-t", session_name])

    # Create a new session with first agent (detached)
    subprocess.call([
        "tmux", "new-session", "-d", "-s", session_name, esp1_cmd
    ])

    # Split window horizontally for second agent
    subprocess.call([
        "tmux", "split-window", "-h", "-t", session_name, esp2_cmd
    ])
    
    # Split window horizontally for third agent
    subprocess.call([
        "tmux", "split-window", "-h", "-t", session_name, esp3_cmd
    ])

    # Set even horizontal layout
    subprocess.call([
        "tmux", "select-layout", "-t", session_name, "even-horizontal"
    ])

    # Attach to the session
    subprocess.call([
        "tmux", "attach-session", "-t", session_name
    ])

    print("✅ 3 ESP32 micro-ROS agents started in tmux session:", session_name)
    print("   - euro/esp1 on /dev/euro/esp1")
    print("   - euro/esp2 on /dev/euro/esp2")
    print("   - euro/esp3 on /dev/euro/esp3")

if __name__ == "__main__":
    main()