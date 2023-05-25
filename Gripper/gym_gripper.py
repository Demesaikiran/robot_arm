
import gym
import numpy as np
from gym.spaces import Box

class RobotArm:
    """
    This class represents a robot arm with four joints and two gripper joints.

    Attributes:
        joint_angles: The joint angles of the robot arm.
        gripper_positions: The gripper positions of the robot arm.
    """

    def __init__(self):
        # Initialize the joint angles.
        self.joint_angles = np.zeros(4)

        # Initialize the gripper positions.
        self.gripper_positions = np.zeros(2)

    def reset(self):
        # Reset the joint angles to their initial position.
        self.joint_angles = np.zeros(4)

        # Reset the gripper positions to their initial position.
        self.gripper_positions = np.zeros(2)

    def apply_action(self, action):
        # Apply the action to the robot arm.
        for i in range(4):
            self.joint_angles[i] += action[i]

    def is_gripper_closed(self, target_object):
        # Check if the gripper is closed on the target object.
        return np.linalg.norm(self.gripper_positions - target_object.position) < 0.01

    def get_joint_angles(self):
        # Get the joint angles of the robot arm.
        return self.joint_angles

    def get_gripper_positions(self):
        # Get the gripper positions of the robot arm.
        return self.gripper_positions

class TargetObject:
    """
    This class represents a target object that can be picked up by the robot arm.

    Attributes:
        position: The position of the target object.
        radius: The radius of the target object.
    """

    def __init__(self):
        # Initialize the position of the target object.
        self.position = np.random.uniform(-1, 1, size=(3,))

        # Initialize the radius of the target object.
        self.radius = 0.1

    def random_position(self):
        # Randomly place the target object in the environment.
        self.position = np.random.uniform(-1, 1, size=(3,))

    def is_picked_up(self, robot_arm):
        # Check if the target object is picked up by the robot arm.
        return np.linalg.norm(self.position - robot_arm.gripper_positions) < self.radius



class RobotArmEnv(gym.Env):
    """
    This environment represents a robot arm with four joints and two gripper joints.

    State space:
        The state space is a 10-dimensional vector. The first 8 dimensions represent the joint angles, and the last 2 dimensions represent the gripper positions.

    Action space:
        The action space is a 10-dimensional vector. The first 8 dimensions represent the desired joint angles, and the last 2 dimensions represent the desired gripper positions.

    Reward function:
        The reward function is +1 if the gripper is closed on the target object, and -1 otherwise.
    """

    def __init__(self):
        super().__init__()

        # Define the state space.
        self.state_space = Box(low=-np.pi, high=np.pi, shape=(8,))

        # Define the action space.
        self.action_space = Box(low=-np.pi, high=np.pi, shape=(8,))

        # Initialize the robot arm.
        self.robot_arm = RobotArm()

        # Initialize the target object.
        self.target_object = TargetObject()

    def reset(self):
        # Reset the robot arm to its initial position.
        self.robot_arm.reset()

        # Randomly place the target object in the environment.
        self.target_object.random_position()

        return self.get_state()

    def step(self, action):
        # Apply the action to the robot arm.
        self.robot_arm.apply_action(action)

        # Check if the gripper is closed on the target object.
        if self.robot_arm.is_gripper_closed(self.target_object):
            reward = 1
        else:
            reward = -1

        # Get the next state of the robot arm.
        next_state = self.get_state()

        return next_state, reward, False, {}

    def get_state(self):
        # Get the joint angles and gripper positions of the robot arm.
        joint_angles = self.robot_arm.get_joint_angles()
        gripper_positions = self.robot_arm.get_gripper_positions()

        # Return the state as a 10-dimensional vector.
        return np.concatenate([joint_angles, gripper_positions])

