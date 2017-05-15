# RL_walking
Implements a learning Intelligent agent using Reinforcement Learning and ES algorithms to control a walking robot.


# Code organization
## Cart-Pole (to be put in gym/examples)
- q_learning_cart: runs Q-learning to solve the Cart-Pole problen
- test.py: does nothing, old q-learning template
- template_q_learning: old template which could be useful (not used)

## Walking Robot
### Agents and implementations (to be put in gym/examples)
- biped_solved: OpenAI Gym walking robot simulation solved with ES alg.
- model-pedal1.p: Saved model of the trained ES alg. learning agent 
- model-pedal_COPY.p: Copy of the model to avoid erasing when training
- sim_biBot: simulation of trained agent on modified env. before testing
### Environments (to be put in gym/gym/envs/box2d)
- bipedal_walker: slightly modified bipedal_walker to remove many variables
- walkbot: modified bipedal_walker to simulate real-world robot
### Real-world implementation on a robot (to be used wherever)
- model-pedal1.p: Saved an copied model of the trained ES alg.
- rl_walking.py: old implementation
- test_angles.py: Newest implementation of the ES alg. to control a walking
	robot. To be used after having trained the model and saved
	model-pdeal1.p in the same folder. All information in the 
	report refer to this file.

## To run the code
### Connect robot to computer using FT232RL or similar circuit
- upload the code from 'walkingBot'
### Run main python script on computer:
~/code$ ./test_angles.py 


# Simulation, installation and OpenAI Gym
## Basic installation
https://github.com/openai/gym
## Modified environments
To install the modified environment used in this Git (available in box2d folder), please follow this link:
- https://github.com/openai/gym/wiki/Environments
and add in gym/envs/__init__.py:

    register(
        id='WalkBot-v0',
        entry_point='gym.envs.box2d:WalkBot',
        max_episode_steps=1600,
        reward_threshold=300,
    )

OpenAI Gym environment is used to train the agent:
BipedalWalker-v2
- https://gym.openai.com/envs/BipedalWalker-v2
- https://github.com/openai/gym/blob/master/gym/envs/box2d/bipedal_walker.py

  State consists of hull angle speed, angular velocity, horizontal speed, vertical speed,
  position of joints and joints angular speed, legs contact with ground, and 10 lidar
  rangefinder measurements to help to deal with the hardcore version. There's no coordinates
  in the state vector. Lidar is less useful in normal version, but it works.

We omit the Lidar for now and only reproduce the walking pattern on an even terrain.

Example of a generic agent running the cross-entropy method: https://github.com/openai/gym/blob/master/examples/agents/cem.py


# Additionnal ressources
### Final report
A final report of the project at the end of the Spring 2017 semester is available in this rep.
see "RL_walking_report.pdf"

### ES algorithm 
https://blog.openai.com/evolution-strategies/

### Angles definitions for Box2D
- http://www.iforce2d.net/b2dtut/joints-revolute

### How to choose learning rate additionnal ressources:
- http://www.jmlr.org/papers/volume5/evendar03a/evendar03a.pdf 
- http://stackoverflow.com/questions/33011825/learning-rate-of-a-q-learning-agent

In the end, we chose a different simpler approach
which we tested empirically


### About Q-learning for the walking part
- We wrote a Q-learning code for the walking part
	but we deleted it because it wasn't working and wasnt a good idea
  

# Future improvements
First, touch sensing of the legs of the robot need to be implemented.
Then, data integration from the sensors needs to be improved (Kalman Filter?)
I plan on running the main code on a Raspberry Pi 3 B running Linux and write a ROS node running the Python code.

