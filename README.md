# RL_walking
Implements a Reinforcement Learning algorithm to control a walking robot.

Simulation
----------

OpenAI Gym environment is used to train the agent.
BipedalWalker-v2
https://gym.openai.com/envs/BipedalWalker-v2
https://github.com/openai/gym/blob/master/gym/envs/box2d/bipedal_walker.py

State consists of hull angle speed, angular velocity, horizontal speed, vertical speed,
position of joints and joints angular speed, legs contact with ground, and 10 lidar
rangefinder measurements to help to deal with the hardcore version. There's no coordinates
in the state vector. Lidar is less useful in normal version, but it works.

We omit the Lidar for now and only reproduce the walking pattern on an even terrain.

Real-world implementation
-------------------------

Using a Raspberry Pi 3 B running Linux and ROS with our Python code, we reproduce the agent policy to walk in real-time.

