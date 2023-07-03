import numpy as np

class UR5Kinematics:
    def __init__(self):
        # UR5 parameters
        self.d1 = 0.089159
        self.a2 = -0.42500
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
    
    def forward_kinematics(self, theta):
        # Denavit-Hartenberg parameters - constants
        alpha = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]
        a = [0, 0, self.a2, self.a3, 0, 0]
        d = [self.d1, 0, 0, self.d4, self.d5, self.d6]
    
        # Homogeneous transformation matrix
        T = np.eye(4)

        for i in range(6):
            ct = np.cos(theta[i])
            st = np.sin(theta[i])
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
        
            A = np.array([[ct, -st*ca, st*sa, a[i]*ct],
                          [st, ct*ca, -ct*sa, a[i]*st],
                          [0, sa, ca, d[i]],
                          [0, 0, 0, 1]])
            T = np.matmul(T, A)
    
        return T

    def inverse_kinematics(self, T):
        theta = [0, 0, 0, 0, 0, 0]
        
        # Denavit-Hartenberg parameters
        alpha = [0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]
        a = [0, 0, self.a2, self.a3, 0, 0]
        d = [self.d1, 0, 0, self.d4, self.d5, self.d6]
        
        # Cálculo tetha 1 
        p05 = np.dot(T, np.array([[0],[0],[-self.d6],[1]]))  # origin 5 with respect to 1
        p05_x = p05[0][0]
        p05_y = p05[1][0]  
        p05_xy = (p05_x**2 + p05_y**2)**0.5
        phi_1 = np.arctan2(p05_y, p05_x)
        phi_2 = np.arccos(self.d4 / p05_xy)
        theta1 = phi_1 + phi_2 + np.pi/2  # SHOULDER LEFT

        # Assign the calculated joint angles to theta list
        theta[0] = theta1
        
        # Cálculo tetha 5
        p06_x = tf[0][3]
        p06_y = tf[1][3]
        num = p06_x*sin(self._theta1) - p06_y*cos(self._theta1) - self.__d4
        if abs(num) >= self.__d6 and num > 0:
            self._theta5 = 0        
        elif abs(num) >= self.__d6 and num < 0:
            self._theta5 = pi
        else:
            self._theta5 = acos(num/self.__d6)

        if wrist == 'up':
            self._theta5 *= -1
    
        return theta

        