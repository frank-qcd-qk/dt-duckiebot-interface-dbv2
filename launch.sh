#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

echo "Starting pigpiod."
pigpiod

roslaunch duckiebot_interface_dbv2 all_drivers.launch veh:=$VEHICLE_NAME robot_type:=$ROBOT_TYPE
