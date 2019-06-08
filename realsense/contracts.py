from lazycontract import LazyContract, LazyProperty, ListProperty, FloatProperty, IntProperty

class BinaryProperty(LazyProperty):
    _type = bytes
    def deserialize(self, obj):
        return obj if isinstance(obj, self._type) else None

# Contracts for publishing to streams
class IntrinsicsStreamContract(LazyContract):
    STREAM_NAME = "intrinsics"

    width = IntProperty(required=True)
    height = IntProperty(required=True)
    ppx = IntProperty(required=True)
    ppy = IntProperty(required=True)
    fx = IntProperty(required=True)
    fy = IntProperty(required=True)

class AccelStreamContract(LazyContract):
    STREAM_NAME = "accel"

    x = IntProperty(required=True)
    y = IntProperty(required=True)
    z = IntProperty(required=True)

class GyroStreamContract(LazyContract):
    STREAM_NAME = "gyro"

    x = IntProperty(required=True)
    y = IntProperty(required=True)
    z = IntProperty(required=True)

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

    x = IntProperty(required=True)
    y = IntProperty(required=True)
    z = IntProperty(required=True)
    qx = FloatProperty(required=True)
    qy = FloatProperty(required=True)
    qz = FloatProperty(required=True)
    qw = FloatProperty(required=True)
