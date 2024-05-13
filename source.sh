#!/bin/bash

export TURTLEBOT3_MODEL=burger
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:../driving_swarm_infrastructure/src/driving_swarm_bringup/models/:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:../driving_swarm_infrastructure/src/driving_swarm_bringup/worlds/:/opt/ros/humble/share/turtlebot3_gazebo/models
source /opt/ros/humble/local_setup.bash
source ../driving_swarm_infrastructure/install/local_setup.bash
source ./install/local_setup.bash
export PYTHONWARNINGS=ignore:::setuptools.command.install

