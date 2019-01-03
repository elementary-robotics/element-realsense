## realsense

### Overview
The realsense element obtains data from a realsense device and publishes the color, depth, and pointcloud data on a stream.
A static transformation between the camera and world is published on a stream and can be updated by following the transform calculation procedure.

### Commands
| Command Name           | Data | Response |
| ---------------------- | ---- | -------- |
| calculate_transform    | None | None     |


### Streams
| Stream                 | Format            |
| ---------------------- | ----------------- |
| color                  | TIF encoded image |
| depth                  | TIF encoded image |
| pointcloud             | TIF encoded image |
| intrinsics             | float             |
| transform              | float             |


### Decoding the image streams
The element writes TIF encoded images to the `color`, `depth`, and `pointcloud` streams.
They can be decoded by performing the following procedure.
```
color_data = element.entry_read_n("realsense", "color", 1)
try:
    color_data = color_data[0]["data"]
except IndexError or KeyError:
    raise Exception("Could not get data. Is the realsense element running?")
color_img = cv2.imdecode(np.frombuffer(color_data, dtype=np.uint8), -1)
```

### Static Transform Calculation
If you would like to use the realsense camera from a static position and convert camera coordinates to world coordinates, you can calculate the transform between the camera space and world space. 

First, let's estimate the transform.
1. Place the [checkerboard](https://raw.githubusercontent.com/elementary-robotics/element-realsense/master/data/checkerboard.png?token=AHgfVAAEjyS03qR-Gb_E2a8Q39t65juDks5b7J0ywA%3D%3D) within the field of view of the realsense camera.
2. Send the `calculate_transform` command to the `realsense` element.

Now the `transform` stream will broadcast an xyz translation with a quaternion. You can convert camera coordinates to world coordinates by performing the following.

1. Convert the quaternion to a rotation matrix
2. Perform a dot product between the camera coordinates and the rotation matrix
3. Add the xyz translation to the result in step 2.

### Attribution
The realsense element is based on [this example](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py) from Intel.
