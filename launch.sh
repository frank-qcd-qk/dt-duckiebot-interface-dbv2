#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# If the container is stopped while pigpiod is running, it will fail to clean up the /var/run/pigpio.pid file.
# So, on each start of the container, we need to do the following:

PIDFILE=/var/run/pigpio.pid

# If the pid file exists:
if [ -f "$PIDFILE" ]; then
  # Display the PID for logging purposes (probably not actually important)
  echo "$PIDFILE exists. Contents:"
  cat $PIDFILE
  
  # Kill the process if it is running (it is probably not actually running, but better safe than sorry)
  if kill -0 $(cat $PIDFILE) 2>/dev/null; then
    kill $(cat $PIDFILE)
  fi

  # Now that we know the old process isn't running, we can remove the file.
  rm $PIDFILE
fi

echo "Starting pigpiod."
pigpiod

roslaunch duckiebot_interface_dbv2 all_drivers.launch veh:=$VEHICLE_NAME robot_type:=$ROBOT_TYPE
