#!/usr/bin/env python3

import gym
from gym import spaces
import numpy as np
import json
import time
import math
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

class Policy(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super(Policy, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, output_dim)
        self.log_std = nn.Parameter(torch.zeros(output_dim))

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        mean = self.fc3(x)
        std = self.log_std.exp()
        return mean, std

class TurtlebotEnv(gym.Env):
    def __init__(self):
        super(TurtlebotEnv, self).__init__()
        
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(365,), dtype=np.float32)
        
        self.goal = {'x': 0, 'y': 0}
        self.set_new_goal()

    def set_new_goal(self):
        self.goal['x'] = np.random.uniform(-5, 5)
        self.goal['y'] = np.random.uniform(-5, 5)

    def get_robot_state(self):
        try:
            with open('/tmp/turtlebot_state.json', 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print("Warning: turtlebot_state.json not found. Returning default state.")
            return {'x': 0, 'y': 0, 'yaw': 0, 'laser_ranges': [5.0] * 360}
        except json.JSONDecodeError:
            print("Warning: Error decoding turtlebot_state.json. Returning default state.")
            return {'x': 0, 'y': 0, 'yaw': 0, 'laser_ranges': [5.0] * 360}

    def send_robot_action(self, linear, angular):
        try:
            with open('/tmp/turtlebot_action.json', 'w') as f:
                json.dump({'linear': float(linear), 'angular': float(angular)}, f)
        except Exception as e:
            print("Warning: Failed to write action to file. Error: {}".format(e))

    def step(self, action):
        linear_vel = action[0] * 0.5
        angular_vel = action[1] * 1.5
        
        self.send_robot_action(linear_vel, angular_vel)
        
        time.sleep(0.1)
        
        state = self.get_robot_state()
        
        distance_to_goal = math.sqrt((self.goal['x'] - state['x'])**2 + (self.goal['y'] - state['y'])**2)
        angle_to_goal = math.atan2(self.goal['y'] - state['y'], self.goal['x'] - state['x']) - state['yaw']
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi
        
        reward = -distance_to_goal
        if distance_to_goal < 0.2:
            reward += 100
            self.set_new_goal()
        
        if min(state['laser_ranges']) < 0.3:
            reward -= 50
        
        done = False
        if min(state['laser_ranges']) < 0.2:
            done = True
            reward -= 100
        
        obs = np.concatenate([
            [state['x'], state['y'], state['yaw']],
            [distance_to_goal, angle_to_goal],
            state['laser_ranges']
        ])
        
        return obs, reward, done, {}

    def reset(self):
        self.send_robot_action(0, 0)
        self.set_new_goal()
        time.sleep(1)
        
        state = self.get_robot_state()
        distance_to_goal = math.sqrt((self.goal['x'] - state['x'])**2 + (self.goal['y'] - state['y'])**2)
        angle_to_goal = math.atan2(self.goal['y'] - state['y'], self.goal['x'] - state['x']) - state['yaw']
        angle_to_goal = (angle_to_goal + np.pi) % (2 * np.pi) - np.pi
        
        obs = np.concatenate([
            [state['x'], state['y'], state['yaw']],
            [distance_to_goal, angle_to_goal],
            state['laser_ranges']
        ])
        
        return obs

def train():
    env = TurtlebotEnv()
    policy = Policy(365, 128, 2)
    optimizer = optim.Adam(policy.parameters(), lr=3e-4)

    num_episodes = 1000
    max_steps = 1000

    for episode in range(num_episodes):
        state = env.reset()
        episode_reward = 0

        for step in range(max_steps):
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            mean, std = policy(state_tensor)
            action_dist = Normal(mean, std)
            action = action_dist.sample()
            action = action.detach().numpy().squeeze()

            next_state, reward, done, _ = env.step(action)
            episode_reward += reward

            # PPO update (simplified)
            optimizer.zero_grad()
            log_prob = action_dist.log_prob(torch.FloatTensor(action)).sum()
            loss = -log_prob * reward
            loss.backward()
            optimizer.step()

            if done:
                break

            state = next_state

        print("Episode {}, Reward: {}".format(episode, episode_reward))

    torch.save(policy.state_dict(), "turtlebot_policy.pth")

def test():
    env = TurtlebotEnv()
    policy = Policy(365, 128, 2)
    policy.load_state_dict(torch.load("turtlebot_policy.pth"))
    policy.eval()

    num_episodes = 10
    max_steps = 1000

    for episode in range(num_episodes):
        state = env.reset()
        episode_reward = 0

        for step in range(max_steps):
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            mean, _ = policy(state_tensor)
            action = mean.detach().numpy().squeeze()

            next_state, reward, done, _ = env.step(action)
            episode_reward += reward

            if done:
                break

            state = next_state

        print("Test Episode {}, Reward: {}".format(episode, episode_reward))

if __name__ == '__main__':
    train()
    test()
