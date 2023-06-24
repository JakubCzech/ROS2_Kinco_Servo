#!/bin/bash
docker build -t agv_servo:local .
xhost +
docker run -it --rm --name=agv_servo_local \
--ulimit memlock=-1 \
--privileged \
--device=/dev/:/dev/:rw \
--env="SERVO_PORT=$SERVO_PORT" \
--volume $(pwd)/src_files/src:/root/workspace/src:rw \
--device=/dev/*:/dev/*:rw \
--network=host \
agv_servo:local bash
