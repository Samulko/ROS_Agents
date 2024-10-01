#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/samko/Documents/GitHub/ros_noetic_311/venv_ros_noetic/bin/activate
source /home/samko/Documents/GitHub/ros_noetic_311/devel/setup.bash
python3 $(rospack find planning_agent)/scripts/planning_agent.py
