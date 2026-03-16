#!/bin/bash

   # -v /home/universe/git/InternNav:/workspace/InternNav \
   # -v /home/universe/git/InternUtopia:/workspace/InternUtopia \
   #-v /media/TrainDataset/InternData-N1:/workspace/InternNav/data \

xhost +local:root
docker run --name gd_mapping --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
   -e "PRIVACY_CONSENT=Y" \
   -e DISPLAY \
   -w /gd_mapping \
   -v $HOME/.Xauthority:/root/.Xauthority \
   -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
   -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
   -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
   -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
   -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
   -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
   -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
   -v ~/docker/isaac-sim/documents:/root/Documents:rw \
   -v /home/universe/gd_project/modules/gd_mapping/workspace:/gd_mapping \
   -v /home/universe/data:/datasets \
   dnwn24/gs-sdf:torch2.9.0-cuda13.0-issac-ros2-jazzy-5090
