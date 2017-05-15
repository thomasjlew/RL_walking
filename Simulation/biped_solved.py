#!/usr/bin/env python
"""
BIPED_SOLVED - Applies an ES algorithm on a walking robot simulation
                to walk successfuly.

Ressources: 
Salimans, T., Ho, J., Chen, X., and Sutskever, I. (2017).
Evolution strategies as a scalable alternative to
reinforcement learning. arXiv preprint arXiv:1703.03864

Requirements:
    OpenAI Gym with Box2D installed
    cPickle
    numpy

Author:   Thomas Lew
email:    thomas.lew@epfl.ch
Website:  https://github.com/thomasjlew/
May 2017; Last revision: 15-May-2017
"""


# Adapted from http://codegists.com/search/openai%20bipedalwalker/5
# ES BipedalWalker-v2
# converges at around iter 200 in 15 minutes
# for testing model set reload=True
 
import gym
import numpy as np
import cPickle as pickle
import sys

NUM_EPISODES = 10001
 
env = gym.make('BipedalWalker-v2')
np.random.seed(10)
 
hl_size = 100   # hidden layer  deep RL: http://karpathy.github.io/2016/05/31/rl/
version = 1
npop = 50
sigma = 0.1
alpha = 0.01
iter_num = 300
aver_reward = None
allow_writing = True
reload = False      #   To reload and visualize simulation, use 'True'
#reload = True
PRINT_ACTIONS = reload
 
print(hl_size, version, npop, sigma, alpha, iter_num)
 
if reload:
    model = pickle.load(open('model-pedal%d.p' % version, 'rb'))
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
        state, reward, done, info = env.step(action)
        angle0 = round(state[4],4)
        angle1 = round(state[6],4)
        angle2 = round(state[9],4)
        angle3 = round(state[11],4)

        if PRINT_ACTIONS:
            print "angle (0,1,2,3): (" + str(angle0) + "," + str(angle1) + "," \
                                       + str(angle2) + "," + str(angle3) + ")"
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