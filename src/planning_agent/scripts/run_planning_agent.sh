#!/bin/bash
source /home/samko/Documents/GitHub/ros_noetic_311/venv_ros_py310/bin/activate
exec python3 /home/samko/Documents/GitHub/ros_noetic_311/src/planning_agent/scripts/planning_agent.py "$@"