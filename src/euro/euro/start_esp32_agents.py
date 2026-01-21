import subprocess, os, sys, time

def run(cmd):
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

bashrc = os.path.expanduser("~/.bashrc")
agent_cmd = "ros2 run micro_ros_agent micro_ros_agent"

esp_cmds = [
    f"bash -c 'source {bashrc} && {agent_cmd} serial --dev /dev/ttyUSB0 -b 115200'",
    f"bash -c 'source {bashrc} && {agent_cmd} serial --dev /dev/ttyUSB1 -b 115200'",
    f"bash -c 'source {bashrc} && {agent_cmd} serial --dev /dev/ttyACM0 -b 115200'",
]

session = "esp32_agents"

run(["tmux", "kill-session", "-t", session])
run(["tmux", "new-session", "-d", "-s", session, esp_cmds[0]])
time.sleep(0.2)

run(["tmux", "split-window", "-h", "-t", session, esp_cmds[1]])
time.sleep(0.1)

run(["tmux", "split-window", "-h", "-t", session, esp_cmds[2]])
run(["tmux", "select-layout", "-t", session, "even-horizontal"])

subprocess.call(["tmux", "attach", "-t", session])