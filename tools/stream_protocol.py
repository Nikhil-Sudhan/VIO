"""Versioned local binary transport for the ROS-free OpenVINS stream adapter."""
import math
import struct

MAGIC = b'PIVIO001'
HEADER = struct.Struct('<BQQQI')
IMU = struct.Struct('<6d')
GEOMETRY = struct.Struct('<II')


def imu_packet(timestamp_ns, receipt_ns, sequence, values):
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected six finite SI inertial measurements')
    return HEADER.pack(1,timestamp_ns,receipt_ns,sequence,IMU.size)+IMU.pack(*values)


def camera_packet(timestamp_ns, receipt_ns, sequence, pixels):
    if pixels.ndim != 2 or pixels.dtype.name != 'uint8':
        raise ValueError('Expected uint8 grayscale pixels')
    height,width = pixels.shape
    return HEADER.pack(2,timestamp_ns,receipt_ns,sequence,8+width*height)+GEOMETRY.pack(width,height)+pixels.tobytes()


def end_packet(): return HEADER.pack(3,0,0,0,0)
