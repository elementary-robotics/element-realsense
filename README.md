## realsense

### Build Status

[![CircleCI](https://circleci.com/gh/elementary-robotics/element-realsense.svg?style=svg&circle-token=381050538b00f6e885ccbc54eb327165be627eef)](https://circleci.com/gh/elementary-robotics/element-realsense)

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


### docker-compose configuration
To give our container access to the realsense device over USB, we must pass `privileged: true`

```yaml
  realsense:
    image: elementaryrobotics/element-realsense
    volumes:
      - type: volume
        source: shared
        target: /shared
        volume:
          nocopy: true
    depends_on:
      - "nucleus"
    privileged: true
    environment:
      - "ROTATION=0"
```
The rotation of the color and depth images can be configured through the `ROTATION` variable in the `environment` section, where the value is the rotation of the image in degrees that is a multiple of 90.


### Decoding the image streams
The element writes TIF encoded images to the `color`, `depth`, and `pointcloud` streams.
They can be decoded by performing the following procedure.

```python
color_data = element.entry_read_n("realsense", "color", 1)
try:
    color_data = color_data[0]["data"]
except IndexError or KeyError:
    raise Exception("Could not get data. Is the realsense element running?")
color_img = cv2.imdecode(np.frombuffer(color_data, dtype=np.uint8), -1)
```


### Data Format
| Image Type |    Size   | Data Type |       Unit      |
| ---------- | --------- | --------- | --------------- |
| color      | 480x640x3 | uint8     |  Intensity      |
| depth      | 480x640x1 | uint16    | Distance in mm  |
| pointcloud | 307200x3  | float32   | Distance in m   |


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
