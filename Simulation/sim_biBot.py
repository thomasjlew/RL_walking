#!/usr/bin/env python

"""
SIM_BIBOT - Runs a ES algorithm on a modified simulation of a walking robot.
Used to visualize the trained results before implementing in the real robot.

To train the agent, it is advised to train the agent on the usual simulation
(using biped_solved.py) and to reload the trained model to test it on this sim.
Justifications of this approach can be found in the final report.
""" 
# Adapted from http://codegists.com/search/openai%20bipedalwalker/5
# https://blog.openai.com/evolution-strategies/
# from ES BipedalWalker-v2
# converges at around iter 200 in 15 minutes

# Runs WalkBot, converges using an ES algoritm
#  this is an implementation of this ES algorithm for a walking robot
 
import gym
import numpy as np
import cPickle as pickle
import sys

#   To train the agent, select
#reload = False
#   To show the simulation with trained agent, select
reload = True
PRINT_ACTIONS = reload # it rather prints the angles
PRINT_ANGLES = reload

#   NOTE: To test the trained agent on the real-world robot, use test_robot.py

NUM_EPISODES = 10001
 
#env = gym.make('BipedalWalker-v2')
env = gym.make('WalkBot-v0')

""" ------- Variables and Convetions of the Model --------
state = [
    self.hull.angle,        # Normal angles up to 0.5
    2.0*self.hull.angularVelocity/FPS,
    0.3*vel.x*(VIEWPORT_W/SCALE)/FPS,  # Normalized to get -1..1 range
    0.3*vel.y*(VIEWPORT_H/SCALE)/FPS,
    self.joints[0].angle,
    self.joints[0].speed / SPEED_HIP,
    self.joints[1].angle + 1.0,
    self.joints[1].speed / SPEED_KNEE,
    1.0 if self.legs[1].ground_contact else 0.0,
    self.joints[2].angle,
    self.joints[2].speed / SPEED_HIP,
    self.joints[3].angle + 1.0,
    self.joints[3].speed / SPEED_KNEE,
    1.0 if self.legs[3].ground_contact else 0.0
    ]

#   actions[i] in (-1,1)
#   Robot angles and axis defintion (index is action[i])
#          ______                    TORQUES DIRECTIONS
#         |______>                 <--.   |   --.
#  [i]:    0/  \2                   +1 )  | ^ -1 )
#          /    \ (light color)    \  /   |  \  /
#  (dark)  \    /    (GREEN)        --    |   --
#   (RED)  1\  /3        
#   1 & 3: maximum angle is vertical, minimum is -90 [deg]


NEW VERSION: THE INDICES INDICATE THE ANGLES. ALL GOOD
#          ______                    TORQUES DIRECTIONS
#         |______>                 <--.   |   --.
#  [i]:    2/  \0                   +1 )  | ^ -1 )
#          /    \                  \  /   |  \  /
#          \    /    (RED)          --    |   --
# (GREEN)  3\  /1        
#   1 & 3: maximum angle is vertical, minimum is -90 [deg]

-------------------------------------------------------------------  """



np.random.seed(10)
 
hl_size = 100
version = 1
npop = 50
sigma = 0.1
alpha = 0.01
iter_num = 300
aver_reward = None
allow_writing = True
 
print(hl_size, version, npop, sigma, alpha, iter_num)
 
if reload:
    model = pickle.load(open('model-pedal%d.p' % version, 'rb'))
    #model = pickle.load(open('model-pedal_COPY.p', 'rb'))
else:
    model = {}      #    len(state) == 24
    #hull angle speed                   1
    #angular velocity                   1
    #horizontal speed                   1
    #vertical speed                     1
    #position of joints                 4
    #joints angular speed               4
    #legs contact with ground           2
    #10 lidar rangefinder measurements  10
    model['W1'] = np.random.randn(24, hl_size) / np.sqrt(24)
    model['W2'] = np.random.randn(hl_size, 4) / np.sqrt(hl_size)
 
def get_action(state, model):
    hl = np.matmul(state, model['W1'])
    hl = np.tanh(hl)
    action = np.matmul(hl, model['W2'])
    action = np.tanh(action)
    return action
 
def f(model, render=False):
#def f(model, render=True):
    state = env.reset()
    total_reward = 0
    for t in range(iter_num):
        if render: env.render()
        action = get_action(state, model) 
        
        #action[0] = 0.9
        #action[1] = 0
        #action[2] = 0
        #action[3] = 0
        #print "Actions: "
        #print action
        state, reward, done, info = env.step(action)
        #http://www.iforce2d.net/b2dtut/joints-revolute:
        #Note that the joint angle increases as 
        #bodyB rotates counter-clockwise in relation to bodyA
        #AGNLES IN RAD
        # if leg is vertical (like at the start), the angle equals zero
        angle0 = round(state[4],4) #front leg, up
        angle1 = round(state[6],4)-1 #for some reason, the state is increased
        angle2 = round(state[9],4)   # by one for lower part of legs angles
        angle3 = round(state[11],4)-1
        hull_angle = round(state[0],4)
        hull_omega = round(state[1],4)
        hull_v_x = round(state[2],4) #between -0.2 and 0 and 0.5
        hull_v_y = round(state[3],4) #between -0. and 0.3
        speed_servo_0 = round(state[5],4)

        PRINT_ACTIONS = False
        if PRINT_ACTIONS:
            print "angle (0,1,2,3): (" + str(angle0) + "," + str(angle1) + "," \
                                       + str(angle2) + "," + str(angle3) + ")"
        if PRINT_ANGLES:
            print "Hull State: (Angle, Omega, V_x, V_y): (" + str(hull_angle) + "," + str(hull_omega) + "," \
                                       + str(hull_v_x) + "," + str(hull_v_y) + ")"
        print "speed: " + str(speed_servo_0)
        print "np.sign speed: " + str(np.sign(speed_servo_0))
        
        #print state[4]
        total_reward += reward
        if done:
            break
    return total_reward
 
if reload:
    iter_num = 10000
    for i_episode in range(10):
        print(f(model, True))
    sys.exit('demo finished')
 
#for episode in range(NUM_EPISODES):
for episode in range(NUM_EPISODES):
    N = {}
    for k, v in model.iteritems(): N[k] = np.random.randn(npop, v.shape[0], v.shape[1])
    R = np.zeros(npop)
    for j in range(npop):
        model_try = {}
        for k, v in model.iteritems(): model_try[k] = v + sigma*N[k][j]
        R[j] = f(model_try)
    A = (R - np.mean(R)) / np.std(R)
    for k in model: model[k] = model[k] + alpha/(npop*sigma) * np.dot(N[k].transpose(1, 2, 0), A)
    if episode % 10 == 0 and allow_writing:
        pickle.dump(model, open('model-pedal%d.p' % version, 'wb'))
    cur_reward = f(model)
    aver_reward = aver_reward * 0.9 + cur_reward * 0.1 if aver_reward != None else cur_reward
    print('episode %d, cur_reward %.2f, aver_reward %.2f' % (episode, cur_reward, aver_reward))