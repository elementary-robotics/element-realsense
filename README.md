### Overview
The realsense element obtains data from a realsense device and publishes the rgb, depth, and pointcloud data on a stream.
A static transformation between the camera and world is published on a stream and can be updated by following the transform calculation procedure.

### Streams
| Element Name | Stream Name            | Format            |
| ------------ | ---------------------- | ----------------- |
| realsense    | color                  | TIF encoded image |
| realsense    | depth                  | TIF encoded image |
| realsense    | pointcloud             | TIF encoded image |
| realsense    | intrinsics             | Dict              |
| realsense    | transform              | Dict              |

### Commands
| Element Name | Command Name           | Data |
| ------------ | ---------------------- | ---- |
| realsense    | calculate_transform    | None |


### Transform Calculation Procedure
1. Place the [checkerboard](https://raw.githubusercontent.com/elementary-robotics/element-realsense/master/data/checkerboard.png?token=AHgfVAAEjyS03qR-Gb_E2a8Q39t65juDks5b7J0ywA%3D%3D) within the field of view of the realsense camera.
2. Send the `calculate_transform` command to the `realsense` element.


### Attribution
The realsense element is based on [this example](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py) from Intel.
