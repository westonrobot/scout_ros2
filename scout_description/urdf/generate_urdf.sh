#!/bin/bash

ros2 run xacro xacro scout_v2.xacro > scout_v2.urdf
ros2 run xacro xacro scout_mini.xacro > scout_mini.urdf