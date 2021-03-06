# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/ray/ROS/tcpip_map_driver/devel;/home/ray/ROS/open_slam/devel;/home/ray/ROS/rm_serial_driver/devel;/home/ray/ROS/catkin_pionner/devel;/home/ray/ROS/xyi2_ws/devel;/home/ray/ROS/turtlebot3_ws/devel;/home/ray/ROS/cartographer_ws/install_isolated;/home/ray/rplidar_driver/devel;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ray/ROS/turtlebot3_ws/src/navigation/dwa_local_planner/cmake-build-debug/devel/env.sh')

output_filename = '/home/ray/ROS/turtlebot3_ws/src/navigation/dwa_local_planner/cmake-build-debug/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
