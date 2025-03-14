#!/bin/bash

USERNAME="itbdelabo"

docker run --rm -it \
  --network host \
  -v $PWD/Certificates:/home/$USERNAME/ros-web-ui-ws/src/ros-web-ui/source/dependencies/aws_mqtt/certs/ \
  -v /home/ubuntu/ros_maps:/ros_maps \
  -e MAPS_FOLDER="/ros_maps" \
  ros-container