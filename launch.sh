#!/bin/bash

python3 realsense/stream.py &
python3 realsense/transform_publisher.py
