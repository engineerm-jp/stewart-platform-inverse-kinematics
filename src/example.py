from stewart_platform_kinematics import StewartPlatformKinematics
import numpy as np

# Example usage
s = StewartPlatformKinematics()
s.plot_stewart_platform(P_P_PO=np.array([-1.01, 0, -0.03]), ROT_P_PO=np.array([10,10,10]), is_deg=True)

# Find the lengths for the linear actuators, and/or the servo angles
L, theta, _, _ = s.inverse_kinematics(np.array([-1, 0, 0]), np.array([0,0,0]), is_deg=True)