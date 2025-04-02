#!/bin/bash

USERNAME="itbdelabo"

docker run --rm -it \
  --network host \
  -v $PWD/Certificates:/home/$USERNAME/ros-web-ui-ws/src/dependencies/aws_mqtt/certs/ \
  -v $PWD/source:/home/$USERNAME/ros-web-ui-ws/src/ \
  -v /home/ubuntu/ros_maps:/home/ubuntu/ros_maps \
  -e MAPS_FOLDER="/ros_maps" \
  ros-container \
  bash -c "export ROS_DISTRO=noetic && \
          export ROS_PYTHON_VERSION=3 && \
          source /opt/ros/noetic/setup.bash && \
          cd /home/$USERNAME/ros-web-ui-ws && \
          sudo rosdep install --from-paths src --ignore-src -r -y && \
          catkin_make && \
          cd src/dependencies/ROS-dashboard-backend/scripts/ && \
          npm install && \
          source /home/$USERNAME/ros-web-ui-ws/devel/setup.bash && \
          cd ~/ros-web-ui-ws/ && \
          exec bash"