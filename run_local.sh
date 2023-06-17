#!/bin/bash
docker build -t agv_servo:local .
xhost +
docker run -it --rm --name=agv_servo_local \
--ulimit memlock=-1 \
--privileged \
--device=/dev/:/dev/:rw \
--env="DISPLAY=$DISPLAY" \
--env="SERVO_PORT=$SERVO_PORT" \
--volume $(pwd)/src_files/src:/root/workspace/src:rw \
--device=/dev/ttyUSB0:/dev/ttyUSB0:rw \
--network=host \
agv_servo:local bash
