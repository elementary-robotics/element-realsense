from lazycontract import LazyContract, LazyProperty, FloatProperty, IntegerProperty
from atom.contracts import BinaryProperty, RawContract


REALSENSE_ELEMENT_NAME = "realsense"

# Contracts for commands
class CalculateTransformCommand:
    COMMAND_NAME = "calculate_transform"

    class Request(RawContract):
        SERIALIZE = False

    class Response(RawContract):
        SERIALIZE = False

# Contracts for publishing to streams
class IntrinsicsStreamContract(LazyContract):
    STREAM_NAME = "intrinsics"
    SERIALIZE = True

    width = IntegerProperty(required=True)
    height = IntegerProperty(required=True)
    ppx = IntegerProperty(required=True)
    ppy = IntegerProperty(required=True)
    fx = IntegerProperty(required=True)
    fy = IntegerProperty(required=True)

class AccelStreamContract(LazyContract):
    STREAM_NAME = "accel"
    SERIALIZE = True

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)

class GyroStreamContract(LazyContract):
    STREAM_NAME = "gyro"
    SERIALIZE = True

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)

class ColorStreamContract(LazyContract):
    STREAM_NAME = "color"
    SERIALIZE = False

    data = BinaryProperty(required=True)

class DepthStreamContract(LazyContract):
    STREAM_NAME = "depth"
    SERIALIZE = False

    data = BinaryProperty(required=True)

class PointCloudStreamContract(LazyContract):
    STREAM_NAME = "pointcloud"
    SERIALIZE = False

    data = BinaryProperty(required=True)

class TransformStreamContract(LazyContract):
    STREAM_NAME = "transform"
    SERIALIZE = True

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)
    qx = FloatProperty(required=True)
    qy = FloatProperty(required=True)
    qz = FloatProperty(required=True)
    qw = FloatProperty(required=True)
