from gymnasium import gym
import numpy as np
import mujoco

class RobotArmEnv(gym.Env):
    """A gym environment for a robotic arm."""

    def __init__(self):
        print("hello")
        # Define the observation space.
        self.observation_space = gym.spaces.Box(
            low=np.array([-np.pi, -np.pi, -np.pi, 0, 0, 0]),
            high=np.array([np.pi, np.pi, np.pi, 1, 1, 1]),
        )

        # Define the action space.
        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1, -1]),
            high=np.array([1, 1, 1, 1]),
        )

        # Initialize the environment.
        self.model = mujoco.MjModel.from_xml_path("gripper.xml")
        self.sim = mujoco.MjSim(self.model)
        self.viewer = mujoco.MjViewer(self.sim)

        # Get the joint positions.
        self.joint_positions = self.sim.get_joint_positions()

        # Get the joint velocities.
        self.joint_velocities = self.sim.get_joint_velocities()

        # Get the end effector position.
        self.end_effector_position = self.sim.get_end_effector_position()

        # Get the end effector velocity.
        self.end_effector_velocity = self.sim.get_end_effector_velocity()

    # def __init__(self):
    #     # Define the observation space.
    #     self.observation_space = gym.spaces.Box(
    #         low=np.array([-np.pi, -np.pi, -np.pi, 0, 0, 0]),
    #         high=np.array([np.pi, np.pi, np.pi, 1, 1, 1]),
    #     )

    #     # Define the action space.
    #     self.action_space = gym.spaces.Box(
    #         low=np.array([-1, -1, -1]),
    #         high=np.array([1, 1, 1]),
    #     )

    #     # Initialize the environment.
    #     self.model = mujoco.load_model("gripper.xml")
    #     print(self.model)
    #     self.sim = mujoco.MjSim(self.model)
    #     self.viewer = mujoco.MjViewer(self.sim)

    #     # Get the joint positions.
    #     self.joint_positions = self.sim.get_joint_positions()

    #     # Get the joint velocities.
    #     self.joint_velocities = self.sim.get_joint_velocities()

    #     # Get the end effector position.
    #     self.end_effector_position = self.sim.get_end_effector_position()

    #     # Get the end effector velocity.
    #     self.end_effector_velocity = self.sim.get_end_effector_velocity()

    def reset(self):
        """Reset the environment to its initial state."""
        self.sim.reset()
        self.viewer.reset()

    def step(self, action):
        """Take an action and return the next observation, reward, done, and info."""
        # Apply the action to the environment.
        self.sim.step(action)

        # Get the next observation.
        observation = self.sim.get_state()

        # Get the reward.
        reward = 0

        # Check if the episode is done.
        done = False

        # Get the info.
        info = {}

        return observation, reward, done, info

    def render(self):
        """Render the environment to the screen."""
        self.viewer.render()

        # Wait for the user to press a key.
        self.viewer.waitKey(1)


if __name__ == "__main__":
    # Create an instance of the environment.
    env = RobotArmEnv()
    print("hello")

    # Start training your reinforcement learning agent!
    agent = ...

    for i in range(1000):
        # Get an action from the agent.
        action = agent.act(env.observation)

        # Take an action in the environment.
        observation, reward, done, info = env.step(action)

        # Update the agent's policy.
        agent.update(observation, action, reward, done, info)

        # Check if the episode is done.
        if done:
            break

        # Render the environment.
        env.render()

        # Print the final reward.
        print(reward)