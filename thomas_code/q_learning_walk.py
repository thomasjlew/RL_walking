#!/usr/bin/env python

import gym
import numpy as np
import random
import math
from time import sleep
#	Q-learning: https://en.wikipedia.org/wiki/Q-learning
#	other interesting ressource: https://github.com/vmayoral/basic_reinforcement_learning/blob/master/tutorial4/README.md


## Initialize the "Cart-Pole" environment
env = gym.make('BipedalWalker-v2')

## Defining the environment related constants

# Number of discrete states (bucket) per state dimension
#	note that for x and x', only one value is possible (0)
#	for theta, 6 possibilities of angles are allowed
NUM_BUCKETS = (1, 1, 6, 3)  # (x, x', theta, theta')
#NUM_BUCKETS = (1, 1, 6, 3)  # (x, x', theta, theta')
# Number of discrete actions
NUM_ACTIONS = env.action_space.n # (left, right)
# Bounds for each discrete state
STATE_BOUNDS = list(zip(env.observation_space.low, env.observation_space.high))
#print STATE_BOUNDS
#[(-4.7999999999999998, 4.7999999999999998), (-3.4028234663852886e+38, 3.4028234663852886e+38)
#			-x_max          x_max                    -x_dot_max               x_dot_max 
#(-0.41887902047863906, 0.41887902047863906), (-3.4028234663852886e+38, 3.4028234663852886e+38)]
#			-theta_max     theta_max                  -theta_max_dot           theta_max_dot
STATE_BOUNDS[1] = [-0.5, 0.5]
STATE_BOUNDS[3] = [-math.radians(50), math.radians(50)]
# Index of the action
ACTION_INDEX = len(NUM_BUCKETS)

## Creating a Q-Table for each state-action pair
q_table = np.zeros(NUM_BUCKETS + (NUM_ACTIONS,))
print "NUM_ACTIONS" + str(NUM_ACTIONS)
print "NUM_BUCKETS" + str(NUM_BUCKETS)
print "q_table"
print q_table

## Learning related constants
MIN_EXPLORE_RATE = 0.01
MIN_LEARNING_RATE = 0.1

## Defining the simulation related constants
NUM_EPISODES = 1000
MAX_T = 250
STREAK_TO_END = 120
SOLVED_T = 199
DEBUG_MODE = True

def simulate():
	## Instantiating the learning related parameters
	learning_rate = get_learning_rate(0)
	explore_rate = get_explore_rate(0)
	discount_factor = 0.99  # since the world is unchanging

	num_streaks = 0

	for episode in range(NUM_EPISODES):

		# Reset the environment
		obv = env.reset()

		# the initial state
		print "observation"
		print obv
		state_0 = state_to_bucket(obv)
		print "state_to_bucket"
		print state_0

		for t in range(MAX_T):
			env.render()

			# Select an action
			action = select_action(state_0, explore_rate)

			# Execute the action
			obv, reward, done, _ = env.step(action)

			# Observe the result
			state = state_to_bucket(obv)

			# Update the Q based on the result
			best_q = np.amax(q_table[state])
			q_table[state_0 + (action,)] += learning_rate*(reward + discount_factor*(best_q) - q_table[state_0 + (action,)])

			# Setting up for the next iteration
			state_0 = state

			# Print data
			if (DEBUG_MODE):
				print("\nEpisode = %d" % episode)
				print("t = %d" % t)
				print("Action: %d" % action)
				print("State: %s" % str(state))
				print("Reward: %f" % reward)
				print("Best Q: %f" % best_q)
				print("Explore rate: %f" % explore_rate)
				print("Learning rate: %f" % learning_rate)
				print("Streaks: %d" % num_streaks)

				print("")

			if done:
			   print("Episode %d finished after %f time steps" % (episode, t))
			   if (t >= SOLVED_T):
				   num_streaks += 1
			   else:
				   num_streaks = 0
			   break

			#sleep(0.25)

		# It's considered done when it's solved over 120 times consecutively
		if num_streaks > STREAK_TO_END:
			break

		# Update parameters
		explore_rate = get_explore_rate(episode)
		learning_rate = get_learning_rate(episode)


def select_action(state, explore_rate):
	# Select a random action
	#	random.random() generates a random float uniformly in the semi-open range [0.0, 1.0)
	if random.random() < explore_rate:
		action = env.action_space.sample()
	# Select the action with the highest q
	else:
		action = np.argmax(q_table[state])
	return action


def get_explore_rate(t):
	#expl_rate = 1/float(1+t)
	expl_rate = 1/float(1+t)
	expl_rate = 0.8
	if t > 10:
		expl_rate = 0.7
	if t > 20:
		expl_rate = 0.6
	if t > 30:
		expl_rate = 0.5
	if t > 40:
		expl_rate = 0.4
	if t > 50:
		expl_rate = 0.2
	if t > 60:
		expl_rate = MIN_EXPLORE_RATE


	return max(MIN_EXPLORE_RATE, expl_rate)
	#return max(MIN_EXPLORE_RATE, min(1, 1.0 - math.log10((t+1)/25)))

def get_learning_rate(t):
	#	how to choose learning rate?
	#	http://www.jmlr.org/papers/volume5/evendar03a/evendar03a.pdf 
	#	http://stackoverflow.com/questions/33011825/learning-rate-of-a-q-learning-agent
	#omega = float(3./4.)
	omega = float(1.1/2.)

	#alpha = 1/float(1+t)
	alpha = 1/(float(1+t))**omega
	alpha = 0.8
	if t > 20:
		alpha = 0.7
	if t > 40:
		alpha = 0.6
	if t > 60:
		alpha = 0.5

	alpha = 0.5
	# omega = 1 is a linear learning rate
	# omega = range( 1/2 , 1 ) is a polynomial learning rate
	return max(MIN_LEARNING_RATE, alpha)

	#old function
	#return max(MIN_LEARNING_RATE, min(0.5, 1.0 - math.log10((t+1)/25)))

#	Convert "continuous" values of state into countainers to limit state size
def state_to_bucket(state):
	bucket_indice = []
	for i in range(len(state)):
		if state[i] <= STATE_BOUNDS[i][0]:	#if the value of state is too small
			bucket_index = 0 				#set to value 0 (first possiblity of bucket)
		elif state[i] >= STATE_BOUNDS[i][1]:#if too big
			bucket_index = NUM_BUCKETS[i] - 1 #set to maximum indice (number of countainers)
		else:
			# Mapping the state bounds to the bucket array
			bound_width = STATE_BOUNDS[i][1] - STATE_BOUNDS[i][0]
			offset = (NUM_BUCKETS[i]-1)*STATE_BOUNDS[i][0]/bound_width
			scaling = (NUM_BUCKETS[i]-1)/bound_width
			#change state value to a value between 0 and NUM_BUCKETS[i]-1
			bucket_index = int(round(scaling*state[i] - offset)) 
		bucket_indice.append(bucket_index)
	return tuple(bucket_indice)

if __name__ == "__main__":
	#print math.log10(1/25)
	simulate()