#!/usr/bin/env python3
import subprocess

# Define the commands you want to run in each tab

terminal1_commands = ["ros2 launch t"]
commands_lists = [
    ["ls -l", "pwd", "echo 'Hello, world!'"],
    ["date", "whoami", "echo 'This is another tab!'"]
]

# Create a list of commands to open new tabs and run the specified commands in each tab
tab_commands = []

for commands in commands_lists:
    tab_commands.append("gnome-terminal")
    for command in commands:
        tab_commands.append("--tab 'bash -c \"" + command + "; exec bash\"'")

# Launch GNOME Terminal with tabs for each command list
subprocess.Popen(" ".join(tab_commands), shell=True)

