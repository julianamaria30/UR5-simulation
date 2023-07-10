import sim
import numpy as np
import time
from transforms3d.euler import euler2mat

# UR5 SImulation control
class UR5Simulation:
    def __init__(self):
        print ('Program started')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
        if self.clientID!=-1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server')
            print ('Program ended')

    def get_joint_angles(self):
        # Get joint handles
        joint_handles = []
        for i in range(1, 7):
            _, handle = sim.simxGetObjectHandle(self.clientID, 'UR5_joint' + str(i), sim.simx_opmode_blocking)
            joint_handles.append(handle)

        # Get joint angles from the simulator
        joint_angles = []
        for i in range(6):
            _, angle = sim.simxGetJointPosition(self.clientID, joint_handles[i], sim.simx_opmode_blocking)
            joint_angles.append(angle)

        joint_angles = np.round(joint_angles, decimals=5)

        return joint_angles
    
    def set_joint_angles(self, angles):
        # Get joint handles
        joint_handles = []
        for i in range(1, 7):
            _, handle = sim.simxGetObjectHandle(self.clientID, 'UR5_joint' + str(i), sim.simx_opmode_blocking)
            joint_handles.append(handle)

        # Set joint angles in the simulator
        for i in range(6):
            sim.simxSetJointTargetPosition(self.clientID, joint_handles[i], angles[i], sim.simx_opmode_blocking)

        sim.simxGetPingTime(self.clientID)

    def get_end_effector_pose(self):
        _, handle = sim.simxGetObjectHandle(self.clientID, 'UR5_joint6', sim.simx_opmode_blocking)

        # Enable streaming mode for position and orientation updates
        sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(self.clientID, handle, -1, sim.simx_opmode_streaming)

        # Wait for the first update
        sim.simxGetPingTime(self.clientID)
        time.sleep(0.05)
        # Retrieve the position and orientation
        _, position = sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_buffer)
        _, orientation = sim.simxGetObjectOrientation(self.clientID, handle, -1, sim.simx_opmode_buffer)

        position = np.round(position, decimals=5)
        orientation = np.round(orientation, decimals=5)

        # Convert Euler angles to rotation matrix
        rotation_matrix = euler2mat(orientation[0], orientation[1], orientation[2])

        matrix = np.eye(4)
        matrix[:3, 3] = position
        matrix[:3, :3] = rotation_matrix

        print(matrix)

        return matrix

    def set_end_effector_pose(self, matrix):
        # Set the desired Cartesian pose
        _, ur5_handle = sim.simxGetObjectHandle(self.clientID, 'UR5_joint6', sim.simx_opmode_blocking)

        # Extract position and orientation from the matrix
        position = matrix[:3, 3]
        orientation = matrix[:3, :3]

        # Convert the orientation matrix to Euler angles
        eulerAngles = self.calculate_euler(matrix)

        # Set the end effector pose
        sim.simxSetObjectPosition(self.clientID, ur5_handle, -1, position, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(self.clientID, ur5_handle, -1, eulerAngles, sim.simx_opmode_oneshot)

        sim.simxGetPingTime(self.clientID)

    def get_pose(self):
        _, handle = sim.simxGetObjectHandle(self.clientID, 'UR5_joint6', sim.simx_opmode_blocking)

        # Get the end effector pose
        _, position = sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_blocking)
        _, euler_angles = sim.simxGetObjectOrientation(self.clientID, handle, -1, sim.simx_opmode_blocking)

        # Convert Euler angles to rotation matrix
        rotation_matrix = euler2mat(euler_angles[0], euler_angles[1], euler_angles[2])

        # Create the 4x4 matrix
        matrix = np.eye(4)
        matrix[:3, 3] = position
        matrix[:3, :3] = rotation_matrix

        return matrix

    def calculate_euler(self, matrix):
        orientation = matrix[:3, :3]

        # Calculate Euler angles from the rotation matrix
        sy = np.sqrt(orientation[0, 0] ** 2 + orientation[1, 0] ** 2)

        if sy > 1e-6:
            # Non-singular case (rotation around Z-axis and Y-axis)
            yaw = np.arctan2(orientation[1, 0], orientation[0, 0])
            pitch = np.arctan2(-orientation[2, 0], sy)
            roll = np.arctan2(orientation[2, 1], orientation[2, 2])
        else:
            # Singular case (rotation around X-axis)
            yaw = np.arctan2(-orientation[1, 2], orientation[1, 1])
            pitch = np.arctan2(-orientation[2, 0], sy)
            roll = 0.0

        euler_angles = [roll, pitch, yaw]

        return euler_angles
    
    def sim_sincronization(self):
        _, movement_signal = sim.simxGetStringSignal(self.clientID, 'movement_signal', sim.simx_opmode_blocking)
        return movement_signal

if __name__ == '__main__':

    simulation = UR5Simulation()
    #q = [90*np.pi/180,90*np.pi/180,-90*np.pi/180,90*np.pi/180,90*np.pi/180,90*np.pi/180]
    q = [0, 0, 0, 0, 0, 0]
    simulation.set_joint_angles(q)
    sim.simxGetPingTime(simulation.clientID)
    time.sleep(5)
    print(simulation.get_pose())
    q = [90*np.pi/180,90*np.pi/180,-90*np.pi/180,90*np.pi/180,90*np.pi/180,90*np.pi/180]
    simulation.set_joint_angles(q)
    sim.simxGetPingTime(simulation.clientID)
    time.sleep(5)
    tf = np.array([[-0.00729703, -0.01207085, 0.99990052, -0.09918797],
               [-0.14835317, -0.98884873, -0.01302008, -0.03713172],
               [0.98890752, -0.14843342, 0.00542491, 0.98463148],
               [0., 0., 0., 1.]])
    simulation.set_end_effector_pose(tf)
    sim.simxGetPingTime(simulation.clientID)
    time.sleep(2)