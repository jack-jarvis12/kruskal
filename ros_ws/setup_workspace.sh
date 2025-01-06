#!/bin/bash

# Source the ROS 2 workspace setup script
source install/setup.bash

# Source the Python virtual environment
source src/voice_commands/src/voice_commands/venv/bin/activate

# Print confirmation message
echo "ROS 2 workspace and virtual environment activated."
