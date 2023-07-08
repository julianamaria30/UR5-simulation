import sim
import numpy as np
import time

# UR5 SImulation control
class UR5Simulation:
    def __init__(self):
        print ('Program started')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID=sim.simxStart('127.0.0.1',19996,True,True,5000,5) # Connect to CoppeliaSim
        if self.clientID!=-1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server')
            print ('Program ended')

    def get_joint_angles(self):
        # Get joint handles
        joint_handles = []
        for i in range(1, 7):
            _, handle = sim.simxGetObjectHandle(self.clientID, '/UR5/joint' + str(i), sim.simx_opmode_blocking)
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
            _, handle = sim.simxGetObjectHandle(self.clientID, '/UR5/joint' + str(i), sim.simx_opmode_blocking)
            joint_handles.append(handle)

        # Set joint angles in the simulator
        for i in range(6):
            sim.simxSetJointTargetPosition(self.clientID, joint_handles[i], angles[i], sim.simx_opmode_blocking)

        sim.simxGetPingTime(self.clientID)

    def get_end_effector_pose(self):
        _, handle = sim.simxGetObjectHandle(self.clientID, '/UR5/link7_visible', sim.simx_opmode_blocking)
        _, world_handle = sim.simxGetObjectHandle(self.clientID, '/UR5/link1_visible', sim.simx_opmode_blocking)
        print(world_handle)

        # Enable streaming mode for position and orientation updates
        sim.simxGetObjectPosition(self.clientID, handle, world_handle, sim.simx_opmode_streaming)
        sim.simxGetObjectOrientation(self.clientID, handle, world_handle, sim.simx_opmode_streaming)

        # Wait for the first update
        sim.simxGetPingTime(self.clientID)
        time.sleep(0.05)
        # Retrieve the position and orientation
        _, position = sim.simxGetObjectPosition(self.clientID, handle, world_handle, sim.simx_opmode_buffer)
        _, orientation = sim.simxGetObjectOrientation(self.clientID, handle, world_handle, sim.simx_opmode_buffer)

        position = np.round(position, decimals=5)
        orientation = np.round(orientation, decimals=5)

        print(position)
        print(orientation)

        return position, orientation

    def set_end_effector_pose(self, position, orientation):
        # Set the desired Cartesian pose
        #position = [0.5, 0.5, 0.5]  # X, Y, Z position in meters
        #orientation = [0, 0, 0]  # Roll, pitch, yaw angles in radians

        _, ur5_handle = sim.simxGetObjectHandle(self.clientID, '/UR5/link7_visible', sim.simx_opmode_blocking)
        _, world_handle = sim.simxGetObjectHandle(self.clientID, '/UR5/link1_visible', sim.simx_opmode_blocking)
        print(world_handle)

        sim.simxSetObjectPosition(self.clientID, ur5_handle, world_handle, position, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(self.clientID, ur5_handle, world_handle, orientation, sim.simx_opmode_oneshot)  
        sim.simxGetPingTime(self.clientID) 


if __name__ == '__main__':

    simulation = UR5Simulation()
    q = [0,0,0,0,0,0]
    #simulation.set_joint_angles(q)
    sim.simxGetPingTime(simulation.clientID)
    simulation.get_end_effector_pose()