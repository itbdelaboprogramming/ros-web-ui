#!/bin/bash

USERNAME="itbdelabo"

docker run --rm -it \
  --network host \
  -v $PWD/Certificates:/home/$USERNAME/ros-web-ui-ws/src/dependencies/aws_mqtt/certs/ \
  -v $PWD/source:/home/$USERNAME/ros-web-ui-ws/src/ \
  -v /home/ubuntu/ros_maps:/ros_maps \
  -e MAPS_FOLDER="/ros_maps" \
  ros-container