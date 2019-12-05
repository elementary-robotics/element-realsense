import cv2
import math
import numpy as np
import os
import pyrealsense2 as rs
import subprocess
import sys
import time
from threading import Thread, Lock
from atom import Element
from atom.messages import Response, LogLevel
from contracts import IntrinsicsStreamContract, AccelStreamContract, GyroStreamContract, \
ColorStreamContract, DepthStreamContract, PointCloudStreamContract, TransformStreamContract, \
CalculateTransformCommand, REALSENSE_ELEMENT_NAME


class Realsense:

    def __init__(
        self,
        element_name,
        transform_file_path,
        calibration_client_path,
        depth_shape,
        color_shape,
        fps,
        disparity_shift,
        depth_units,
        rotation,
        retry_delay
    ):
        self._transform_file_path = transform_file_path
        self._calibration_client_path = calibration_client_path
        self._depth_shape = depth_shape
        self._color_shape = color_shape
        self._fps = fps
        self.disparity_shift = disparity_shift
        self.depth_units = depth_units
        self._rotation = rotation
        self._retry_delay = retry_delay

        self._status_is_running = False
        self._status_lock = Lock()
        self._pipeline = rs.pipeline()
        self._rs_pc = rs.pointcloud()
        self._transform = TransformStreamContract(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1)
        self._transform_last_loaded = 0

        # Create an align object: rs.align allows us to perform alignment of depth frames to other frames
        self._align = rs.align(rs.stream.color)

        # Init element
        self._element = Element(element_name)
        self._element.healthcheck_set(self.is_healthy)
        self._element.command_add(
            CalculateTransformCommand.COMMAND_NAME,
            self.run_transform_estimator,
            timeout=2000,
            deserialize=CalculateTransformCommand.Request.SERIALIZE
        )

        # Run command loop
        thread = Thread(target=self._element.command_loop, daemon=True)
        thread.start()

    def is_healthy(self):
        """
        Reports whether the realsense is connected and streaming or not
        """
        try:
            self._status_lock.acquire()
            if self._status_is_running:
                return Response(err_code=0, err_str="Realsense online")
            else:
                return Response(err_code=1, err_str="Waiting for realsense")
        finally:
            self._status_lock.release()

    def load_transform_from_file(self, fname):
        """
        Opens specified file, reads transform, and returns as list.

        Args:
            fname (str): CSV file that stores the transform
        """
        with open(fname, "r") as f:
            transform_list = [float(v) for v in f.readlines()[-1].split(",")]
            return TransformStreamContract(
                x=transform_list[0],
                y=transform_list[1],
                z=transform_list[2],
                qx=transform_list[3],
                qy=transform_list[4],
                qz=transform_list[5],
                qw=transform_list[6]
            )

    def run_transform_estimator(self, *args):
        """
        Runs the transform estimation procedure, which saves the transform to disk.
        """
        process = subprocess.Popen(self._calibration_client_path, stderr=subprocess.PIPE)
        out, err = process.communicate()
        return Response(
            data=CalculateTransformCommand.Response().to_data(),
            err_code=process.returncode,
            err_str=err.decode(),
            serialize=CalculateTransformCommand.Response.SERIALIZE
        )

    def run_camera_stream(self):
        while True:
            try:
                # Try to establish realsense connection
                self._element.log(LogLevel.INFO, "Attempting to connect to Realsense")

                # Set disparity shift
                device = rs.context().query_devices()[0]
                advnc_mode = rs.rs400_advanced_mode(device)
                depth_table_control_group = advnc_mode.get_depth_table()
                depth_table_control_group.disparityShift = self.disparity_shift
                advnc_mode.set_depth_table(depth_table_control_group)

                # Attempt to stream accel and gyro data, which requires d435i
                # If we can't then we revert to only streaming depth and color
                try:
                    config = rs.config()
                    config.enable_stream(
                        rs.stream.depth, self._depth_shape[0], self._depth_shape[1], rs.format.z16, self._fps
                    )
                    config.enable_stream(
                        rs.stream.color, self._color_shape[0], self._color_shape[1], rs.format.bgr8, self._fps
                    )
                    config.enable_stream(rs.stream.accel)
                    config.enable_stream(rs.stream.gyro)
                    profile = self._pipeline.start(config)
                    is_d435i = True
                except RuntimeError:
                    config = rs.config()
                    config.enable_stream(
                        rs.stream.depth, self._depth_shape[0], self._depth_shape[1], rs.format.z16, self._fps
                    )
                    config.enable_stream(
                        rs.stream.color, self._color_shape[0], self._color_shape[1], rs.format.bgr8, self._fps
                    )
                    profile = self._pipeline.start(config)
                    is_d435i = False

                # Set depth units
                depth_sensor = profile.get_device().first_depth_sensor()
                depth_sensor.set_option(rs.option.depth_units, self.depth_units)

                # Publish intrinsics
                rs_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                intrinsics = IntrinsicsStreamContract(
                    width=rs_intrinsics.width,
                    height=rs_intrinsics.height,
                    ppx=rs_intrinsics.ppx,
                    ppy=rs_intrinsics.ppy,
                    fx=rs_intrinsics.fx,
                    fy=rs_intrinsics.fy
                )
                self._element.entry_write(
                    IntrinsicsStreamContract.STREAM_NAME,
                    intrinsics.to_dict(),
                    serialize=IntrinsicsStreamContract.SERIALIZE,
                    maxlen=self._fps
                )

                try:
                    self._status_lock.acquire()
                    self._status_is_running = True
                finally:
                    self._status_lock.release()

                self._element.log(LogLevel.INFO, "Realsense connected and streaming")
                while True:
                    start_time = time.time()

                    frames = self._pipeline.wait_for_frames()
                    aligned_frames = self._align.process(frames)
                    depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()

                    # Validate that frames are valid
                    if not depth_frame or not color_frame:
                        continue

                    # Generate realsense pointcloud
                    self._rs_pc.map_to(color_frame)
                    points = self._rs_pc.calculate(depth_frame)

                    # Convert data to numpy arrays
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    vertices = np.asanyarray(points.get_vertices())

                    vertices = vertices.view(np.float32).reshape(vertices.shape + (-1,))

                    if self._rotation is not None:
                        depth_image = np.rot90(depth_image, k=self._rotation / 90)
                        color_image = np.rot90(color_image, k=self._rotation / 90)
                        # TODO: Apply rotation to pointcloud

                    _, color_serialized = cv2.imencode(".tif", color_image)
                    _, depth_serialized = cv2.imencode(".tif", depth_image)
                    _, pc_serialized = cv2.imencode(".tif", vertices)

                    if is_d435i:
                        accel = frames[2].as_motion_frame().get_motion_data()
                        gyro = frames[3].as_motion_frame().get_motion_data()
                        accel_data = AccelStreamContract(x=accel.x, y=accel.y, z=accel.z)
                        gyro_data = GyroStreamContract(x=gyro.x, y=gyro.y, z=gyro.z)
                        self._element.entry_write(
                            AccelStreamContract.STREAM_NAME,
                            accel_data.to_dict(),
                            serialize=AccelStreamContract.SERIALIZE,
                            maxlen=self._fps
                        )
                        self._element.entry_write(
                            GyroStreamContract.STREAM_NAME,
                            gyro_data.to_dict(),
                            serialize=GyroStreamContract.SERIALIZE,
                            maxlen=self._fps
                        )

                    color_contract = ColorStreamContract(data=color_serialized.tobytes())
                    depth_contract = DepthStreamContract(data=depth_serialized.tobytes())
                    pc_contract = PointCloudStreamContract(data=pc_serialized.tobytes())
                    self._element.entry_write(
                        ColorStreamContract.STREAM_NAME,
                        color_contract.to_dict(),
                        serialize=ColorStreamContract.SERIALIZE,
                        maxlen=self._fps
                    )
                    self._element.entry_write(
                        DepthStreamContract.STREAM_NAME,
                        depth_contract.to_dict(),
                        serialize=DepthStreamContract.SERIALIZE,
                        maxlen=self._fps
                    )
                    self._element.entry_write(
                        PointCloudStreamContract.STREAM_NAME,
                        pc_contract.to_dict(),
                        serialize=PointCloudStreamContract.SERIALIZE,
                        maxlen=self._fps
                    )

                    # Load transform from file if the file exists
                    # and has been modified since we last checked
                    if os.path.exists(self._transform_file_path):
                        transform_last_modified = os.stat(self._transform_file_path).st_mtime
                        if transform_last_modified > self._transform_last_loaded:
                            try:
                                self._transform = self.load_transform_from_file(self._transform_file_path)
                                self._transform_last_loaded = time.time()
                            except Exception as e:
                                self._element.log(LogLevel.ERR, str(e))
                    self._element.entry_write(
                        TransformStreamContract.STREAM_NAME,
                        self._transform.to_dict(),
                        serialize=TransformStreamContract.SERIALIZE,
                        maxlen=self._fps
                    )

                    time.sleep(max(1 / self._fps - (time.time() - start_time), 0))

            except:
                self._element.log(LogLevel.INFO, "Camera loop threw exception: %s" % (sys.exc_info()[1]))
            finally:
                # If camera fails to init or crashes, update status and retry connection
                try:
                    self._status_lock.acquire()
                    self._status_is_running = False
                finally:
                    self._status_lock.release()

                try:
                    self._pipeline.stop()
                except:
                    pass
                time.sleep(self._retry_delay)


if __name__ == "__main__":
    element_name = os.getenv('ELEMENT_NAME', REALSENSE_ELEMENT_NAME)
    transform_file_path = os.getenv('TRANSFORM_FILE_PATH', 'data/transform.csv')
    calibration_client_path = os.getenv('CALIBRATION_CLIENT_PATH', 'build/transform_estimation')
    depth_shape_x = int(os.getenv('DEPTH_SHAPE_X', '640'))
    depth_shape_y = int(os.getenv('DEPTH_SHAPE_Y', '480'))
    color_shape_x = int(os.getenv('COLOR_SHAPE_X', '640'))
    color_shape_y = int(os.getenv('COLOR_SHAPE_Y', '480'))
    disparity_shift = int(os.getenv("DISPARITY_SHIFT", "0"))
    depth_units = float(os.getenv("DEPTH_UNITS", "0.001"))
    fps = int(os.getenv('FPS', '30'))
    retry_delay = float(os.getenv('RETRY_DELAY', '3.0'))
    rotation = os.getenv('ROTATION', None)
    if rotation is not None:
        try:
            rotation = int(rotation)
            if rotation % 90 != 0:
                raise ValueError()
        except ValueError:
            raise ValueError("Rotation must be an integer value that is a multiple of 90!")

    realsense = Realsense(
        element_name=element_name,
        transform_file_path=transform_file_path,
        calibration_client_path=calibration_client_path,
        depth_shape=(depth_shape_x, depth_shape_y),
        color_shape=(color_shape_x, color_shape_y),
        fps=fps,
        disparity_shift=disparity_shift,
        depth_units=depth_units,
        rotation=rotation,
        retry_delay=retry_delay
    )
    realsense.run_camera_stream()
