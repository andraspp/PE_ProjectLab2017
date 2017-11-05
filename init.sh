#!/bin/bash

#############################################
# 1) Help the initialization process someone
# new has checked out the workspace
# -------------------------------------------
# Start the demo world for now
# TODO: Accept args so the user can tell
#   which world he would like to start
#############################################

DIR=$(dirname $0)

if [ -d $DIR/build ]; then
  echo "Workspace already initialized"
else
  echo "Workspace not yet initialized..."
  echo "Starting catkin make..."
  catkin_make
  echo "Finished..."
fi

# Is roscore running?
if ! pgrep -x "roscore" > /dev/null; then
  echo "Roscore is not running :("
  echo "Exiting..."
  exit 1
fi

# Is the model env var correct?
if echo "$GAZEBO_MODEL_PATH" | grep -q "jhonny5"; then
  echo "ENV VAR: GAZEBO_MODEL_PATH is correctly initalized!"
else
  echo "Could not find the correct MODEL_PATH"
  echo "Please initialize GAZEBO_MODEL_PATH by adding the following line to your ~/.bashrc"
  echo "export GAZEBO_MODEL_PATH=$(dirname $(realpath -s $0))/src/jhonny5/models:\${GAZEBO_MODEL_PATH}"
  echo "Exiting..."
  exit 1
fi

# Start up our demo world
source $DIR/devel/setup.bash
roslaunch jhonny5 demo_world.launch
