import cv2
import math
import numpy as np
import os
import pyrealsense2 as rs
import subprocess
import threading
import time
from atom import Element
from atom.messages import Response, LogLevel
from contracts import IntrinsicsStreamContract, AccelStreamContract, GyroStreamContract, \
ColorStreamContract, DepthStreamContract, PointCloudStreamContract, TransformStreamContract, \
CalculateTransformCommand, REALSENSE_ELEMENT_NAME

TRANSFORM_FILE_PATH = "data/transform.csv"
CALIBRATION_CLIENT_PATH = "build/transform_estimation"
DEPTH_SHAPE = (640, 480)
COLOR_SHAPE = (640, 480)
FPS = 30

def load_transform_from_file(fname):
    """
    Opens specified file, reads transform, and returns as list.

    Args:
        fname (str): CSV file that stores the transform
    """
    with open(fname, "r") as f:
        transform_list = [float(v) for v in f.readlines()[-1].split(",")]
        return TransformStreamContract(
            x=transform_list[0], y=transform_list[1], z=transform_list[2],
            qx=transform_list[3], qy=transform_list[4], qz=transform_list[5], qw=transform_list[6]
        )


def run_transform_estimator(*args):
    """
    Runs the transform estimation procedure, which saves the transform to disk.
    """
    process = subprocess.Popen(CALIBRATION_CLIENT_PATH, stderr=subprocess.PIPE)
    out, err = process.communicate()
    return Response(
        data=CalculateTransformCommand.Response().to_data(),
        err_code=process.returncode,
        err_str=err.decode(),
        serialize=CalculateTransformCommand.Response.SERIALIZE)

if __name__ == "__main__":
    try:
        rotation = int(os.environ["ROTATION"])
        if rotation % 90 != 0:
            raise ValueError()
    except ValueError:
        error_msg = "Rotation must be an integer value that is a multiple of 90!"
        element.log(LogLevel.ERR, error_msg)
        raise ValueError(error_msg)
    except KeyError:
        rotation = None

    pipeline = rs.pipeline()

    # Attempt to stream accel and gyro data, which requires d435i
    # If we can't then we revert to only streaming depth and color
    try:
        config = rs.config()
        config.enable_stream(rs.stream.depth, DEPTH_SHAPE[0], DEPTH_SHAPE[1], rs.format.z16, FPS)
        config.enable_stream(rs.stream.color, COLOR_SHAPE[0], COLOR_SHAPE[1], rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        profile = pipeline.start(config)
        is_d435i = True
    except RuntimeError:
        config = rs.config()
        config.enable_stream(rs.stream.depth, DEPTH_SHAPE[0], DEPTH_SHAPE[1], rs.format.z16, FPS)
        config.enable_stream(rs.stream.color, COLOR_SHAPE[0], COLOR_SHAPE[1], rs.format.bgr8, FPS)
        profile = pipeline.start(config)
        is_d435i = False


    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to other frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    rs_pc = rs.pointcloud()
    rs_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics = IntrinsicsStreamContract(
        width=rs_intrinsics.width,
        height=rs_intrinsics.height,
        ppx=rs_intrinsics.ppx,
        ppy=rs_intrinsics.ppy,
        fx=rs_intrinsics.fx,
        fy=rs_intrinsics.fy
    )

    element = Element(REALSENSE_ELEMENT_NAME)
    # Startup command thread
    transform = TransformStreamContract(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1)
    transform_last_loaded = 0
    element.command_add(
        CalculateTransformCommand.COMMAND_NAME,
        run_transform_estimator,
        timeout=2000, deserialize=CalculateTransformCommand.Request.SERIALIZE
    )
    t = threading.Thread(target=element.command_loop, daemon=True)
    t.start()
    element.log(LogLevel.INFO, "Realsense started. Publishing frames.")

    element.entry_write(IntrinsicsStreamContract.STREAM_NAME, intrinsics.to_dict(), serialize=IntrinsicsStreamContract.SERIALIZE, maxlen=FPS)
    try:
        while True:
            start_time = time.time()

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # Validate that frames are valid
            if not depth_frame or not color_frame:
                continue

            # Generate realsense pointcloud
            rs_pc.map_to(color_frame)
            points = rs_pc.calculate(depth_frame)

            # Convert data to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            vertices = np.asanyarray(points.get_vertices())

            vertices = vertices.view(np.float32).reshape(vertices.shape + (-1,))

            if rotation is not None:
                depth_image = np.rot90(depth_image, k=rotation / 90)
                color_image = np.rot90(color_image, k=rotation / 90)
                # TODO: Apply rotation to pointcloud

            _, color_serialized = cv2.imencode(".tif", color_image)
            _, depth_serialized = cv2.imencode(".tif", depth_image)
            _, pc_serialized = cv2.imencode(".tif", vertices)

            if is_d435i:
                accel = frames[2].as_motion_frame().get_motion_data()
                gyro = frames[3].as_motion_frame().get_motion_data()
                accel_data = AccelStreamContract(x=accel.x, y=accel.y, z=accel.z)
                gyro_data = GyroStreamContract(x=gyro.x, y=gyro.y, z=gyro.z)
                element.entry_write(AccelStreamContract.STREAM_NAME, accel_data.to_dict(), serialize=AccelStreamContract.SERIALIZE, maxlen=FPS)
                element.entry_write(GyroStreamContract.STREAM_NAME, gyro_data.to_dict(), serialize=GyroStreamContract.SERIALIZE, maxlen=FPS)

            color_contract = ColorStreamContract(data=color_serialized.tobytes())
            depth_contract = DepthStreamContract(data=depth_serialized.tobytes())
            pc_contract = PointCloudStreamContract(data=pc_serialized.tobytes())
            element.entry_write(ColorStreamContract.STREAM_NAME, color_contract.to_dict(), serialize=ColorStreamContract.SERIALIZE, maxlen=FPS)
            element.entry_write(DepthStreamContract.STREAM_NAME, depth_contract.to_dict(), serialize=DepthStreamContract.SERIALIZE, maxlen=FPS)
            element.entry_write(PointCloudStreamContract.STREAM_NAME, pc_contract.to_dict(), serialize=PointCloudStreamContract.SERIALIZE, maxlen=FPS)

            # Load transform from file if the file exists
            # and has been modified since we last checked
            if os.path.exists(TRANSFORM_FILE_PATH):
                transform_last_modified = os.stat(TRANSFORM_FILE_PATH).st_mtime
                if transform_last_modified > transform_last_loaded:
                    try:
                        transform = load_transform_from_file(TRANSFORM_FILE_PATH)
                        transform_last_loaded = time.time()
                    except Exception as e:
                        element.log(LogLevel.ERR, str(e))
            element.entry_write(
                TransformStreamContract.STREAM_NAME,
                transform.to_dict(),
                serialize=TransformStreamContract.SERIALIZE,
                maxlen=FPS
            )

            time.sleep(max(1 / FPS - (time.time() - start_time), 0))

    finally:
        pipeline.stop()
        del element
