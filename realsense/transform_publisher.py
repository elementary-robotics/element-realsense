import os
import subprocess
import time
import threading
from atom import Element
from atom.messages import Response, LogLevel
from contracts import TransformStreamContract

TRANSFORM_FILE_PATH = "data/transform.csv"
CALIBRATION_CLIENT_PATH = "build/transform_estimation"
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
    return Response(err_code=process.returncode, err_str=err.decode())


if __name__ == "__main__":
    transform_ = TransformStreamContract(x=0, y=0, z=0, qx=0, qy=0, qz=0, qw=1)
    transform_last_loaded = 0

    element = Element("realsense")
    element.command_add("calculate_transform", run_transform_estimator, timeout=2000)
    t = threading.Thread(target=element.command_loop, daemon=True)
    t.start()

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
                        element.log(LogLevel.ERR, str(e))

            element.entry_write(TransformStreamContract.STREAM_NAME, transform.to_dict(), maxlen=FPS)
            time.sleep(max(1/FPS - (time.time() - start_time), 0))

    finally:
        del element
