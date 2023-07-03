import sim
import numpy as np

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
    
    def get_joint_angles(self, client_id, joint_handles):
        # Get joint angles from the simulator
        joint_angles = []
        for i in range(6):
            _, angle = sim.simxGetJointPosition(client_id, joint_handles[i], sim.simx_opmode_blocking)
            joint_angles.append(angle)
        return joint_angles
    
    def set_joint_angles(self, client_id, joint_handles, angles):
        # Set joint angles in the simulator
        for i in range(6):
            sim.simxSetJointTargetPosition(client_id, joint_handles[i], angles[i], sim.simx_opmode_blocking)

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

        p05 = np.dot(T, np.array([[0],[0],[-self.d6],[1]]))  # origin 5 with respect to 1
        p05_x = p05[0][0]
        p05_y = p05[1][0]  
        p05_xy = (p05_x**2 + p05_y**2)**0.5
        phi_1 = np.arctan2(p05_y, p05_x)
        phi_2 = np.arccos(self.d4 / p05_xy)
        theta1 = phi_1 + phi_2 + np.pi/2  # SHOULDER LEFT

        # Assign the calculated joint angles to theta list
        theta[0] = theta1

        return theta


if __name__ == '__main__':
    # Connect to CoppeliaSim
    sim.simxFinish(-1)  # Close any existing connections
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if client_id == -1:
        print("Failed to connect to CoppeliaSim")
        exit()

    # Get joint handles
    joint_handles = []
    for i in range(1, 7):
        _, handle = sim.simxGetObjectHandle(client_id, 'UR5_joint' + str(i), sim.simx_opmode_blocking)
        joint_handles.append(handle)

    # Create UR5 kinematics object
    ur5 = UR5Kinematics()

    # Main control loop
    while sim.simxGetConnectionId(client_id) != -1:
        # Set joint angles to an array of zeros
        angles = np.zeros(6)

        q = [1, 1, 1, 1, 1, 1]

        # Set joint angles in the simulator
        ur5.set_joint_angles(client_id, joint_handles, q)

        # Continue with the simulation
        sim.simxSynchronousTrigger(client_id)

    # Disconnect from CoppeliaSim
    sim.simxFinish(client_id)
    print('Connection closed')