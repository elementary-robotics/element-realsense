from lazycontract import LazyContract, LazyProperty, FloatProperty, IntegerProperty

class BinaryProperty(LazyProperty):
    _type = bytes
    def deserialize(self, obj):
        return obj if isinstance(obj, self._type) else None

# Contracts for publishing to streams
class IntrinsicsStreamContract(LazyContract):
    STREAM_NAME = "intrinsics"

    width = IntegerProperty(required=True)
    height = IntegerProperty(required=True)
    ppx = IntegerProperty(required=True)
    ppy = IntegerProperty(required=True)
    fx = IntegerProperty(required=True)
    fy = IntegerProperty(required=True)

class AccelStreamContract(LazyContract):
    STREAM_NAME = "accel"

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)

class GyroStreamContract(LazyContract):
    STREAM_NAME = "gyro"

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)

class ColorStreamContract(LazyContract):
    STREAM_NAME = "color"

    data = BinaryProperty(required=True)

class DepthStreamContract(LazyContract):
    STREAM_NAME = "depth"

    data = BinaryProperty(required=True)

class PointCloudStreamContract(LazyContract):
    STREAM_NAME = "pointcloud"

    data = BinaryProperty(required=True)

class TransformStreamContract(LazyContract):
    STREAM_NAME = "transform"

    x = IntegerProperty(required=True)
    y = IntegerProperty(required=True)
    z = IntegerProperty(required=True)
    qx = FloatProperty(required=True)
    qy = FloatProperty(required=True)
    qz = FloatProperty(required=True)
    qw = FloatProperty(required=True)
