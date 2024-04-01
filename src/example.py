import numpy as np
from stewart_platform_kinematics import StewartPlatformKinematics

# Example usage
P_P_PO=[-0.01, 0, -0.03] # [x, y, z] relative to the platform origin
ROT_P_PO=[10,10,10] # [roll, pitch, yaw] relative to the platform origin

s = StewartPlatformKinematics()
s.plot_stewart_platform(P_P_PO, ROT_P_PO, is_deg=True)

# Find the lengths for the linear actuators, and/or the servo angles
L, theta, _, _ = s.inverse_kinematics(P_P_PO, ROT_P_PO, is_deg=True)