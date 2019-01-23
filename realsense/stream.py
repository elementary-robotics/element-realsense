import cv2
import numpy as np
import pyrealsense2 as rs
import time
from atom import Element
from atom.messages import Response, LogLevel


DEPTH_SHAPE = (640, 480)
COLOR_SHAPE = (640, 480)
FPS = 15


def rs_intrinsics_to_dict(rs_intrinsics):
    return {
        "width": rs_intrinsics.width,
        "height": rs_intrinsics.height,
        "ppx": rs_intrinsics.ppx,
        "ppy": rs_intrinsics.ppy,
        "fx": rs_intrinsics.fx,
        "fy": rs_intrinsics.fy,
    }


if __name__ == "__main__":
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_SHAPE[0], DEPTH_SHAPE[1], rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, COLOR_SHAPE[0], COLOR_SHAPE[1], rs.format.bgr8, FPS)

    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to other frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    rs_pc = rs.pointcloud()
    rs_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics = rs_intrinsics_to_dict(rs_intrinsics)

    element = Element("realsense")
    element.log(LogLevel.INFO, "Realsense started. Publishing frames.")
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

            _, color_serialized = cv2.imencode(".tif", color_image)
            _, depth_serialized = cv2.imencode(".tif", depth_image)
            _, pc_serialized = cv2.imencode(".tif", vertices)

            element.entry_write("color", {"data": color_serialized.tobytes()}, maxlen=FPS)
            element.entry_write("depth", {"data": depth_serialized.tobytes()}, maxlen=FPS)
            element.entry_write("pointcloud", {"data": pc_serialized.tobytes()}, maxlen=FPS)
            element.entry_write("intrinsics", intrinsics, maxlen=FPS)
            time.sleep(max(1/FPS - (time.time() - start_time), 0))

    finally:
        pipeline.stop()
        del element
