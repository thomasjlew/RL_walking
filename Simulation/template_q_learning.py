#!/usr/bin/env python
#https://vmayoral.github.io/robots,/simulation,/ai,/rl,/reinforcement/learning/2016/08/19/openai-gym-for-robotics/
import random

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))            
        '''
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)

    def chooseAction(self, state, return_q=False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)

        if random.random() < self.epsilon:
            minQ = min(q); mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))] 
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values 
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]        
        if return_q: # if they want it, give it!
            return action, q
        return action

    def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)




import gym
import numpy as numpy
import time
env = gym.make('CartPole-v0')
#env = gym.make('GazeboCircuit2TurtlebotLidar-v0')

outdir = '/tmp/gazebo_gym_experiments'
#env.monitor.start(outdir, force=True, seed=None)
last_time_steps = numpy.ndarray(0)
qlearn = QLearn(actions=range(env.action_space.n),
                alpha=0.2, gamma=0.8, epsilon=0.9)

initial_epsilon = qlearn.epsilon
epsilon_discount = 0.9986
start_time = time.time()
total_episodes = 10000
highest_reward = 0




for x in range(total_episodes):
    done = False
    cumulated_reward = 0
    observation = env.reset()
    if qlearn.epsilon > 0.05:
        qlearn.epsilon *= epsilon_discount
    state = ''.join(map(str, observation))

    for i in range(1500):
        env.render()
        # Pick an action based on the current state
        action = qlearn.chooseAction(state)

        # Execute the action and get feedback
        observation, reward, done, info = env.step(action)
        cumulated_reward += reward

        if highest_reward < cumulated_reward:
            highest_reward = cumulated_reward

        nextState = ''.join(map(str, observation))

        qlearn.learn(state, action, reward, nextState)
        if not(done):
            state = nextState
        else:
            last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
            break 

    m, s = divmod(int(time.time() - start_time), 60)
    h, m = divmod(m, 60)
    print   "EP: " + str(x+1)+ " - [alpha: " + str(round(qlearn.alpha,2)) + \
            " - gamma: " + str(round(qlearn.gamma,2)) + " - epsilon: " + \
            str(round(qlearn.epsilon,2)) + "] - Reward: " + \
            str(cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)

#env.monitor.close()
env.close()

