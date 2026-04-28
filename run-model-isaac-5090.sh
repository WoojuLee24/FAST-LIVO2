#!/bin/bash

xhost +local:root
docker run --name gd_mapping --shm-size 32g --ipc=host --privileged --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
   -e "PRIVACY_CONSENT=Y" \
   -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
   -e DISPLAY \
   -w /gd_mapping \
   -v $HOME/.Xauthority:/root/.Xauthority \
   -v /home/universe/gd_project/modules/gd_mapping/workspace:/gd_mapping \
   -v /home/universe/data:/datasets \
   --ulimit nofile=65536:65536 \
   dnwn24/gs-sdf:torch2.9.0-cuda13.0-issac-ros2-jazzy-5090
