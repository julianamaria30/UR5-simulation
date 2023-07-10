import numpy as np
from math import atan2, acos, pi, sin, cos, isclose, asin, sqrt
from numpy.linalg import inv


# UR5 Kinematics
class UR5Kinematics:
    def __init__(self):
        # UR5 parameters
        self.d1 = 0.089159
        self.a2 = -0.42500
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823

        #self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 = (0,0,0,0,0,0)

    def forward_kinematics(self, theta=[0, 0, 0, 0, 0, 0], link1=0, link2=6):
        T = np.eye(4)
    
        # Denavit-Hartenberg parameters
        alpha = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]
        a = [0, 0, self.a2, self.a3, 0, 0]
        d = [self.d1, 0, 0, self.d4, self.d5, self.d6]
    
        for i in range(link1,link2):
            cos_theta = np.cos(theta[i])
            sin_theta = np.sin(theta[i])
            cos_alpha = np.cos(alpha[i])
            sin_alpha = np.sin(alpha[i])
        
            A =np.array([[cos_theta, -sin_theta, 0, a[i]],
					        [sin_theta*cos_alpha, cos_theta*cos_alpha, -sin_alpha, -sin_alpha*d[i]],
					        [sin_theta*sin_alpha, cos_theta*sin_alpha, cos_alpha, cos_alpha*d[i]],
					        [0, 0, 0, 1]])
            T = np.matmul(T, A)

            # Round the values in matrix T
            T = np.round(T, decimals=5)

        # Extract position and orientation from the transformation matrix
        position = T[:3, 3]
        orientation = np.array([np.arctan2(T[2, 1], T[2, 2]),
                            np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2)),
                            np.arctan2(T[1, 0], T[0, 0])])

        return T

    def inverse_kinematics(self, desired_pos, shoulder = 'left', wrist = 'down', elbow = 'up'):
        P_05 =  np.matmul(desired_pos, np.matrix([[0],[0],[-self.d6],[1]]))

        # **** theta1 ****
        phi1 = atan2(P_05[1,0], P_05[0,0])
        phi2 = acos(self.d4 /sqrt(P_05[1,0]*P_05[1,0] + P_05[0,0]*P_05[0,0]))

        theta1 = pi/2 + phi1 + phi2
        theta1 = np.round(theta1, decimals=5)

        if shoulder == 'right':
            theta1 = pi/2 + phi1 - phi2
            theta1 = np.round(theta1, decimals=5) 

        # **** theta5 ****
        P_06x = desired_pos[0][3]
        P_06y = desired_pos[1][3]
        P_16y = P_06x*sin(theta1) - P_06y*cos(theta1)

        value = (P_16y-self.d4)/self.d6
        value = max(-1, min(value, 1))

        theta5 = acos(value)
        theta5 = np.round(theta5, decimals=5)

        if wrist == 'up':
            theta5 = - acos(value)
            theta5 = np.round(theta5, decimals=5)

        # **** theta6 ****
        T06 = inv(desired_pos) # transformation matrix from 6 to 0
        num_y = -T06[1][0]*sin(theta1) + T06[1][1]*cos(theta1) # y numerator
        num_x = T06[0][0]*sin(theta1) - T06[0][1]*cos(theta1) # x numerator

        if abs(sin(theta5)) < 1e-6:      # indeterminate case
            theta6 = 0.0
        elif num_x == 0 and num_y == 0:
            theta6 = 0.0                 #indeterminate case
        else:
            y = num_y/sin(theta5)
            x = num_x/sin(theta5)
            theta6 = atan2(y, x)

        # **** theta3 ****
        T01 = self.homogeneous_transformation(0, theta1)
        T45 = self.homogeneous_transformation(4, theta5)
        T56 = self.homogeneous_transformation(5, theta6)
        
        T01 = np.round(T01, decimals=5)
        T45 = np.round(T45, decimals=5)
        T56 = np.round(T56, decimals=5)

        T14 = np.dot(np.dot(inv(T01),desired_pos),inv(np.dot(T45,T56))) 
        P14_x = T14[0][3]
        P14_z = T14[2][3]
        P14_xz = sqrt(P14_x*P14_x + P14_z*P14_z)
        value = (P14_xz**2 - self.a2**2 - self.a3**2)/(2*self.a2*self.a3)
        value = max(-1, min(value, 1))

        theta3 = acos(value)
        theta3 = np.round(theta3, decimals=5)

        if elbow == 'down':
            theta3 = - acos(value)
            theta3 = np.round(theta3, decimals=5)

        # **** theta2 ****
        phi_1 = atan2(-P14_z, -P14_x)
        phi_2 = asin((-self.a3*sin(theta3))/P14_xz)
        theta2 = phi_1 - phi_2
        theta2 = np.round(theta2, decimals=5)

        # **** theta4 ****
        T12 = self.homogeneous_transformation(1, theta2)
        T23 = self.homogeneous_transformation(2, theta3)

        T30 = inv(np.dot(np.dot(T01,T12),T23))
        T64 = inv(np.dot(T45,T56))
        T34 = np.dot(np.dot(T30,desired_pos),T64)

        theta4 = atan2(T34[1][0], T34[0][0])        
        theta4 = np.round(theta4, decimals=5)

        return theta1, theta2, theta3, theta4, theta5, theta6

    def homogeneous_transformation(self, i, theta):
        # Denavit-Hartenberg parameters
        alpha = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]
        a = [0, 0, self.a2, self.a3, 0, 0]
        d = [self.d1, 0, 0, self.d4, self.d5, self.d6]

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha[i])
        sin_alpha = np.sin(alpha[i])

        A =np.array([[cos_theta, -sin_theta, 0, a[i]],
					    [sin_theta*cos_alpha, cos_theta*cos_alpha, -sin_alpha, -sin_alpha*d[i]],
					    [sin_theta*sin_alpha, cos_theta*sin_alpha, cos_alpha, cos_alpha*d[i]],
					    [0, 0, 0, 1]])

        return A


if __name__ == '__main__':
    # Create UR5 kinematics object
    ur5 = UR5Kinematics()
    #q = [0,0,0,0,0,0]
    q = [90*np.pi/180,90*np.pi/180,-90*np.pi/180,90*np.pi/180,90*np.pi/180,90*np.pi/180]
    print(ur5.forward_kinematics(q))
    pose = ur5.forward_kinematics(q)
    print(ur5.inverse_kinematics(pose))
    angles = ur5.inverse_kinematics(pose)
    print(ur5.forward_kinematics(angles))


