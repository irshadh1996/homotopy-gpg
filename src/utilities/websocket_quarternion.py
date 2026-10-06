
from scipy.spatial.transform import Rotation
def quaternion_to_euler_yaw(x, y, z, w):
    rotation = Rotation.from_quat([x, y, z, w])
    euler_angles = rotation.as_euler('xyz', degrees=False)
    return euler_angles[2] # Yaw is the third component (z-axis rotation)
