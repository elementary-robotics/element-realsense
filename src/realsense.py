import cv2
import numpy as np
import os
import pyrealsense2 as rs
import time
from multiprocessing import Process
from pyquaternion import Quaternion
from atom import Element
from atom.messages import Response
from subprocess import run


DEPTH_SHAPE = (640, 360)
COLOR_SHAPE = (640, 480)
TRANSFORM_FILE_PATH = "data/transform.csv"
CALIBRATION_CLIENT_PATH = "build/transform_estimation"
FPS = 10


def load_transform_from_file(fname):
    """
    Opens specified file, reads transform, and returns as list.

    Args:
        fname (str): CSV file that stores the transform
    """
    with open(fname, "r") as f:
        transform = [float(v) for v in f.readlines()[-1].split(",")]
        return transform


def run_calibration_client(*args):
    """
    Runs the transform estimation procedure, which saves the transform to disk.
    """
    result = run(CALIBRATION_CLIENT_PATH, capture_output=True)
    return Response(err_code=result.returncode, err_str=result.stderr.decode())
        

def calibrate_vertices(vertices, translation, quaternion):
    """
    Args:
        vertices (numpy array): vertices to be calibrated
        translation (list): translation in the format [x, y, z]
        quaternion (list): quaternion in the format [qx, qy, qz, qw]
    """
    # Pyquaternion expects [qw, qx, qy, qz]
    q = Quaternion(quaternion[-1:]+quaternion[:-1])
    rotation = q.rotation_matrix.astype(np.float32)
    translation = np.array(translation, dtype=np.float32)
    vertices = np.dot(vertices, rotation.T) + translation
    return vertices


if __name__ == "__main__":
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_SHAPE[0], DEPTH_SHAPE[1], rs.format.z16, 15)
    config.enable_stream(rs.stream.color, COLOR_SHAPE[0], COLOR_SHAPE[1], rs.format.bgr8, 15)

    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    rs_pc = rs.pointcloud()
    transform = None
    transform_last_loaded = 0

    element = Element("realsense")
    element.command_add("calibrate", run_calibration_client, timeout=2000)

    p = Process(target=element.command_loop)
    p.start()
    print("Realsense started. Publishing frames.")
    try:
        while True:
            start_time = time.time()

            # Load transform from file if the file exists
            # and has been modified since we last checked
            if os.path.exists(TRANSFORM_FILE_PATH):
                transform_last_modified = os.stat(TRANSFORM_FILE_PATH).st_mtime
                if transform_last_modified > transform_last_loaded:
                    try:
                        transform = load_transform_from_file(TRANSFORM_FILE_PATH)
                        transform_last_loaded = time.time()
                    except Exception as e:
                        print(e)

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

            _, color_serialized = cv2.imencode(".tif", color_image)
            _, depth_serialized = cv2.imencode(".tif", depth_image)
            _, pc_serialized = cv2.imencode(".tif", vertices)

            element.entry_write("color", {"data": color_serialized.tobytes()}, maxlen=FPS)
            element.entry_write("depth", {"data": depth_serialized.tobytes()}, maxlen=FPS)
            element.entry_write("pointcloud", {"data": pc_serialized.tobytes()}, maxlen=FPS)
            if transform:
                vertices = calibrate_vertices(vertices, transform[:3], transform[3:])
                _, pc_serialized = cv2.imencode(".tif", vertices)
                element.entry_write("pointcloud_calibrated", 
                    {"data": pc_serialized.tobytes()}, maxlen=FPS)
            time.sleep(max(1/FPS - (time.time() - start_time), 0))

    finally:
        pipeline.stop()
        p.terminate()
        p.join()
        del element
