from UR5Kinematics import UR5Kinematics
import numpy as np

def main():
    # Instantiate the UR5RobotArm class
    robot = UR5Kinematics()
    
    # Define the joint angles (in radians)
    #joint_angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    
    joint_angles = [0, 0, 0, 0, 0, 0]

    # Calculate the forward kinematics
    end_effector_pose = robot.forward_kinematics(joint_angles)
    print("End Effector Pose:\n", end_effector_pose)
    
    # Define the desired end effector pose
    desired_pose = np.array([[1, 0, 0, 0.5],
                             [0, 1, 0, 0.3],
                             [0, 0, 1, 0.2],
                             [0, 0, 0, 1]])
    
    # Calculate the inverse kinematics
    joint_angles = robot.inverse_kinematics(desired_pose)
    print("Joint Angles:\n", joint_angles)

if __name__ == '__main__':
    main()