import numpy as np
from numpy import sin, cos, pi, sqrt, arctan2
from matplotlib import pyplot as plt

# Stewart Platform Inverse Kinematics Solver
class StewartPlatformKinematics:

    def __init__(self):
        # Convention: P_{i}_{j} = [x, y, z] is the position vector of i relative to j
        # +ve x is forward, +ve y is left, +ve z is up
        
        # --- Platform parameters (customizable) ---
        self.L_B = 0.1    #  Length between the base joints
        self.L_P = 0.05     #  Length between the platform joints
        self.L_bar = 0.12   #  Length of the bars
        self.L_servo = 0.049#  Length of the servo arm

        # Positions of the base joints Bn (n = 0 - 5)) relative to the base origin frame BO
        # Assuming the base is a regular hexagon (customizable for other shapes)
        self.P_Bn_BO = np.array([
            [self.L_B*cos(pi/6),     self.L_B*sin(pi/6), 0],
            [self.L_B*cos(pi/2),     self.L_B*sin(pi/2), 0],
            [self.L_B*cos(5*pi/6),   self.L_B*sin(5*pi/6), 0],
            [self.L_B*cos(7*pi/6),   self.L_B*sin(7*pi/6), 0],
            [self.L_B*cos(3*pi/2),   self.L_B*sin(3*pi/2), 0],
            [self.L_B*cos(11*pi/6),  self.L_B*sin(11*pi/6), 0]
        ]) # 3 x 6
        
        # Rotations of the base joints' axes relative to the base origin frame BO (customizable)
        # Note: the x-axis of the base joint is aligned with the rotation axis of the servo motor
        self.R_BnO_BO = np.array([
            [0, 0, pi/6],
            [0, 0, pi/2],
            [0, 0, 5*pi/6],
            [0, 0, 7*pi/6],
            [0, 0, 3*pi/2],
            [0, 0, 11*pi/6]
        ])
        
        # Positions of the platform joints Pn (n = 0 - 5) relative to the platform origin frame PO
        # Assuming the platform is a regular hexagon but offset by 30 deg around yaw (customizable for other shapes)
        self.P_Pn_P = np.array([
            [self.L_P*cos(pi/6),     self.L_P*sin(pi/6), 0],
            [self.L_P*cos(pi/2),     self.L_P*sin(pi/2), 0],
            [self.L_P*cos(5*pi/6),   self.L_P*sin(5*pi/6), 0],
            [self.L_P*cos(7*pi/6),   self.L_P*sin(7*pi/6), 0],
            [self.L_P*cos(3*pi/2),   self.L_P*sin(3*pi/2), 0],
            [self.L_P*cos(11*pi/6),  self.L_P*sin(11*pi/6), 0]
        ]) # 3 x 6
        
        # Set default platform origin (PO) pose relative to the base origin frame BO
        self.P_PO_BO = np.array([0, 0, self.L_bar+0.03])
        self.R_PO_BO = self.rotation_matrix(np.array([0, 0, 0]))
        
        # --- Platform parameters end ---

    def inverse_kinematics(self, P_P_PO, ROT_P_PO, is_deg=False) -> np.ndarray:
        # Input:    P_P_PO - position vector of the platform origin relative to the initial platform origin
        #           ROT_P_IPO - rotation angles of the platform relative to the initial platform origin
        
        # Check if the input angles are in degrees
        ROT_P_PO = ROT_P_PO * pi/180 if is_deg else ROT_P_PO

        # Find the rotation matrix from the platform frame to the initial platform frame
        R_P_IPO = self.rotation_matrix(ROT_P_PO)    
        # Find the homogeneous transformation matrix from initial platform origin PO to the plat form
        T_P_PO = self.homogeneous_transform(R_P_IPO, P_P_PO)
        
        # Find the homogeneous transformation matrix from the base origin to the initial platform origin
        T_PO_BO = self.homogeneous_transform(self.R_PO_BO, self.P_PO_BO)
        
        # Find the platform joints Pn relative to the base origin BO
        P_Pn_BO = np.array([np.array(T_PO_BO.dot(T_P_PO).dot(np.append(self.P_Pn_P[i], 1))[:3]) for i in range(6)])
        
        # Find the distance between the base joints and the platform joints (for linear actuators)
        L = np.array([np.linalg.norm(P_Pn_BO[i] - self.P_Bn_BO[i]) for i in range(6)])
        
        # For a Stewart Platform with linear actuators, the servo angles are not needed
        angles = self.calculate_servo_angle(P_Pn_BO, self.P_Bn_BO)
        
        return L, angles, P_Pn_BO, self.P_Bn_BO

    def calculate_servo_angle(self, P_Pn_BO, P_Bn_BO) -> np.ndarray: 
        
        try: 
            theta_pair = []

            for i in range(6):
                # Find the rotation matrix and position vector of the base origin (BO) relative to the base joints (Bn)
                # where Bn is the n-th base joint
                R_BO_Bn = np.transpose(self.rotation_matrix(self.R_BnO_BO[i]))
                P_BO_Bn = -R_BO_Bn.dot(P_Bn_BO[i])
                
                # Find the homogeneous transformation of the base origin (BO) relative to the base joint (Bn)
                T_BO_Bn = self.homogeneous_transform(R_BO_Bn, P_BO_Bn)
                
                # Find the position vector of the platform joint (Pn) relative to the base joint (Bn)
                x_Pn_Bn, y_Pn_Bn, z_Pn_Bn = T_BO_Bn.dot(np.append(P_Pn_BO[i], 1))[:3]
                
                # Solve the quadratic equation to find y, z in the base joint (Bn) frame
                D = self.L_bar**2 - self.L_servo**2 - x_Pn_Bn**2 - y_Pn_Bn**2 - z_Pn_Bn**2
                a = 4*y_Pn_Bn**2 + 4*z_Pn_Bn**2
                b = 4*y_Pn_Bn*D
                c = D**2 - 4*z_Pn_Bn**2*self.L_servo**2
                y_Bn = self.quadratic_solver(a, b, c) 

                # Check if there is a solution
                if (y_Bn.size == 0): # No solution (i.e., the platform is out of reach)
                    print("No solution for the quadratic equation")
                    theta_pair.append(np.array([]))
                    continue
                
                theta_Bn = np.array([])
                for y in y_Bn:
                    if y**2 > self.L_servo**2: # No solution (i.e., the platform is out of reach)
                        print("No solution: y^2 > L_servo^2")
                        continue
                    z = sqrt(self.L_servo**2 - y**2)
                    P_Arm_Bn = np.array([0, y, z])
                    
                    # Check if the length of the servo arm is within the physical constraints
                    length = np.linalg.norm(P_Arm_Bn - np.array([x_Pn_Bn, y_Pn_Bn, z_Pn_Bn]))
                    
                    # Check if the z component of the servo arm is negative
                    if abs(length - self.L_bar)/self.L_bar > 0.01: # If the length is not within 1% of the bar length 
                        P_Arm_Bn = np.array([0, y, -z])
                        length = np.linalg.norm(P_Arm_Bn - np.array([x_Pn_Bn, y_Pn_Bn, z_Pn_Bn]))

                    theta_Bn = np.append(theta_Bn, arctan2(z,y))
                
                # If there is only one solution, repeat it
                if theta_Bn.size == 1: 
                    theta_Bn = np.array([theta_Bn, theta_Bn])
                
                theta_pair.append(theta_Bn) 

            # Return the servo angles in radians. Generally, there are two solutions for the servo angle,
            # as the servo arm can be on either side of the platform joint
            theta_servo = np.vstack(theta_pair)
        except: 
            print("Error in calculating the servo angles")
        return theta_servo 

    def plot_stewart_platform(self, P_P_PO, ROT_P_PO, is_deg=False) -> None:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_box_aspect([1,1,1]) 
        
        # Label the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # Check if the input angles are in degrees
        ROT_P_PO = ROT_P_PO * pi/180 if is_deg else ROT_P_PO
        
        try:
            # Find the angles of the servo arms based on the platform pose
            L, theta, P_Pn_BO, P_Bn_BO = self.inverse_kinematics(P_P_PO, ROT_P_PO)

            # Plot the base joints Bn and the platform joints Pn
            P_Bn_BO = np.append(P_Bn_BO, [P_Bn_BO[0]], axis=0) # Close the loop for plotting
            P_Pn_BO = np.append(P_Pn_BO, [P_Pn_BO[0]], axis=0)
            
            ax.plot(P_Bn_BO[:,0], P_Bn_BO[:,1], P_Bn_BO[:,2], 'ro-')
            ax.plot(P_Pn_BO[:,0], P_Pn_BO[:,1], P_Pn_BO[:,2], 'bo-')
        except:
            print("Input pose is out of reach")
            return

        for i in range(6):
            
            ## Plot the lines connecting the base joints to the platform joints (for linear actuators)
            # ax.plot([P_Bn_BO[i,0], P_Pn_BO[i,0]], [P_Bn_BO[i,1], P_Pn_BO[i,1]], [P_Bn_BO[i,2], P_Pn_BO[i,2]], 'g-')
        
            # Plot the servo arm (note: the joints with even indices use the first solution, and the joints with odd indices use the second solution)
            P_Arm_Bn = np.array([0, self.L_servo*cos(theta[i][i%2]), self.L_servo*sin(theta[i][i%2])])
            T_Bn_BO = self.homogeneous_transform(self.rotation_matrix(self.R_BnO_BO[i]), self.P_Bn_BO[i])
            P_Arm_BO = T_Bn_BO.dot(np.append(P_Arm_Bn, 1))[:3]
            ax.plot([P_Bn_BO[i,0], P_Arm_BO[0]], [P_Bn_BO[i,1], P_Arm_BO[1]], [P_Bn_BO[i,2], P_Arm_BO[2]], 'k-')

            # Plot the bar to the platform joint
            ax.plot([P_Arm_BO[0], P_Pn_BO[i,0]], [P_Arm_BO[1], P_Pn_BO[i,1]], [P_Arm_BO[2], P_Pn_BO[i,2]], 'm-')

        plt.show()
        
    def quadratic_solver(self, a, b, c) -> np.ndarray:
        # Solve the quadratic equation ax^2 + bx + c = 0
        D = b**2 - 4*a*c
        if D < 0:
            return np.array([])
        elif D == 0:
            return np.array([-b/(2*a)], [-b/(2*a)])
        else:
            return np.array([(-b + np.sqrt(D))/(2*a), (-b - np.sqrt(D))/(2*a)])

    def rotation_matrix(self, angles) -> np.ndarray:
        roll, pitch, yaw = angles[0], angles[1], angles[2]
        
        # Rotation matrix from the base frame to the platform frame
        R_x = np.array([
            [1, 0, 0],
            [0, cos(roll), -sin(roll)],
            [0, sin(roll), cos(roll)]])    
        
        R_y = np.array([
            [cos(pitch), 0, sin(pitch)],
            [0, 1, 0], 
            [-sin(pitch), 0, cos(pitch)]])
        
        R_z = np.array([
            [cos(yaw), -sin(yaw), 0], 
            [sin(yaw), cos(yaw), 0],
            [0, 0, 1]])
        
        return R_z.dot(R_y.dot(R_x))
    
    def homogeneous_transform(self, R, P) -> np.ndarray:
        # Homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = P
        return T
    

# Example usage
s = StewartPlatformKinematics()
s.plot_stewart_platform(P_P_PO=np.array([0.01, 0, -0.03]), ROT_P_PO=np.array([10,10,10]), is_deg=True)

# Find the lengths for the linear actuators, and/or the servo angles
L, theta, _, _ = s.inverse_kinematics(np.array([0, 0, 0]), np.array([0,0,0]), is_deg=True)