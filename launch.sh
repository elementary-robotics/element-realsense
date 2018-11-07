#!/bin/bash

python3 src/stream.py &
python3 src/transform_publisher.py
