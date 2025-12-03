---
sidebar_position: 6
title: Advanced RL and Sim-to-Real Transfer
description: Master advanced reinforcement learning techniques, multi-agent systems, and sim-to-real transfer strategies for deploying trained policies on physical robots
---

# Advanced RL and Sim-to-Real Transfer

## Prerequisites

Before starting this chapter, you should have:

- ✅ Completed all previous Module 3 chapters (especially Isaac Gym RL)
- ✅ Strong understanding of RL algorithms (PPO, SAC, TD3)
- ✅ Experience training policies in Isaac Gym
- ✅ NVIDIA RTX GPU (12GB+ VRAM for large-scale training)
- ✅ Access to physical robot hardware (for deployment sections)
- ✅ Understanding of control theory and system identification

**Estimated Reading Time**: 30-35 minutes

---

## Introduction

While you've learned to train basic policies in Isaac Gym, **production robotics** requires:
- **Advanced RL algorithms**: Multi-task learning, hierarchical RL, curriculum learning
- **Sim-to-real transfer**: Bridging the reality gap between simulation and physical robots
- **Safety and robustness**: Handling failure modes, constraints, and uncertainty
- **Multi-agent coordination**: Training teams of robots to collaborate

This chapter explores **state-of-the-art techniques** for deploying RL policies in real-world robotics:

**Why Advanced RL?**
- **Sample efficiency**: Learn complex behaviors faster
- **Generalization**: Policies that work across task variations
- **Robustness**: Handle sensor noise, actuator delays, and modeling errors
- **Scalability**: Train multiple skills simultaneously

**Learning Objectives**:
1. Implement advanced RL algorithms (HER, SAC-X, PEARL)
2. Design curriculum learning strategies for complex skills
3. Apply domain randomization and system identification for sim-to-real transfer
4. Deploy trained policies on physical robots with safety guarantees
5. Build multi-agent systems for collaborative manipulation

---

## Advanced RL Algorithms

### Hindsight Experience Replay (HER)

**Problem**: Sparse rewards make learning difficult (e.g., "reach exact position" only rewards success).

**Solution**: HER treats failures as successes for *different* goals.

```python
# her_reach_task.py
from omni.isaac.gym.vec_env import VecEnvBase
import torch
import numpy as np

class HERReachEnv(VecEnvBase):
    def __init__(self, num_envs=4096):
        super().__init__(num_envs=num_envs)
        self.replay_buffer = HERReplayBuffer(capacity=1000000)

    def compute_reward(self, achieved_goal, desired_goal):
        """Compute reward: -1 if not at goal, 0 if at goal"""
        distance = torch.norm(achieved_goal - desired_goal, dim=-1)
        return -(distance > 0.05).float()  # Binary reward

    def step(self, actions):
        # Execute actions
        self.robot.apply_actions(actions)
        self.world.step()

        # Get observations
        achieved_goal = self.robot.get_end_effector_position()
        desired_goal = self.target_position
        obs = self.get_observations()

        # Compute reward
        reward = self.compute_reward(achieved_goal, desired_goal)

        done = (reward == 0) | (self.step_count > 200)

        # Store transition
        self.replay_buffer.add(obs, actions, reward, achieved_goal, desired_goal)

        # HER: Recompute reward for alternative goals
        if done.any():
            self.hindsight_relabel()

        return obs, reward, done, {}

    def hindsight_relabel(self):
        """Relabel past experiences with achieved goals as new goals"""
        batch = self.replay_buffer.sample(batch_size=256)

        for i in range(len(batch)):
            original_obs = batch[i]["obs"]
            original_action = batch[i]["action"]
            achieved_goal = batch[i]["achieved_goal"]

            # Pretend achieved_goal was the desired goal
            new_reward = self.compute_reward(achieved_goal, achieved_goal)  # Always 0 (success)

            # Store relabeled transition
            self.replay_buffer.add(
                original_obs,
                original_action,
                new_reward,
                achieved_goal,
                achieved_goal  # New goal
            )

class HERReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0

    def add(self, obs, action, reward, achieved_goal, desired_goal):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)

        self.buffer[self.position] = {
            "obs": obs,
            "action": action,
            "reward": reward,
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal
        }
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        indices = np.random.randint(0, len(self.buffer), size=batch_size)
        return [self.buffer[i] for i in indices]
```

### Soft Actor-Critic (SAC) for Continuous Control

**SAC** is more sample-efficient than PPO for robotic manipulation:

```python
# sac_manipulation.py
import torch
import torch.nn as nn
import torch.optim as optim

class SACAgent:
    def __init__(self, obs_dim, action_dim, hidden_dim=256):
        # Actor network (policy)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim * 2)  # Mean and log_std
        )

        # Critic networks (Q-functions)
        self.critic1 = self._build_critic(obs_dim, action_dim, hidden_dim)
        self.critic2 = self._build_critic(obs_dim, action_dim, hidden_dim)

        # Target critics (for stability)
        self.critic1_target = self._build_critic(obs_dim, action_dim, hidden_dim)
        self.critic2_target = self._build_critic(obs_dim, action_dim, hidden_dim)
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=3e-4)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=3e-4)

        # Temperature parameter for entropy
        self.log_alpha = torch.tensor([0.0], requires_grad=True)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=3e-4)

        self.gamma = 0.99  # Discount factor
        self.tau = 0.005  # Target network update rate

    def _build_critic(self, obs_dim, action_dim, hidden_dim):
        return nn.Sequential(
            nn.Linear(obs_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def select_action(self, obs, deterministic=False):
        """Sample action from policy"""
        with torch.no_grad():
            mean, log_std = self.actor(obs).chunk(2, dim=-1)
            log_std = torch.clamp(log_std, -20, 2)
            std = log_std.exp()

            if deterministic:
                return torch.tanh(mean)

            # Reparameterization trick
            normal = torch.randn_like(mean)
            action = mean + std * normal
            return torch.tanh(action)

    def update(self, batch):
        """Update SAC agent from batch of transitions"""
        obs, action, reward, next_obs, done = batch

        # Update critics
        with torch.no_grad():
            next_action, next_log_prob = self._sample_action(next_obs)
            q1_target = self.critic1_target(torch.cat([next_obs, next_action], dim=-1))
            q2_target = self.critic2_target(torch.cat([next_obs, next_action], dim=-1))
            q_target = torch.min(q1_target, q2_target) - self.log_alpha.exp() * next_log_prob
            target = reward + self.gamma * (1 - done) * q_target

        q1 = self.critic1(torch.cat([obs, action], dim=-1))
        q2 = self.critic2(torch.cat([obs, action], dim=-1))

        critic1_loss = nn.MSELoss()(q1, target)
        critic2_loss = nn.MSELoss()(q2, target)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # Update actor
        new_action, log_prob = self._sample_action(obs)
        q1_new = self.critic1(torch.cat([obs, new_action], dim=-1))
        q2_new = self.critic2(torch.cat([obs, new_action], dim=-1))
        q_new = torch.min(q1_new, q2_new)

        actor_loss = (self.log_alpha.exp() * log_prob - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Update temperature
        alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()

        # Soft update target networks
        self._soft_update()

    def _sample_action(self, obs):
        """Sample action and compute log probability"""
        mean, log_std = self.actor(obs).chunk(2, dim=-1)
        log_std = torch.clamp(log_std, -20, 2)
        std = log_std.exp()

        normal = torch.randn_like(mean)
        z = mean + std * normal
        action = torch.tanh(z)

        # Log probability with tanh correction
        log_prob = torch.distributions.Normal(mean, std).log_prob(z)
        log_prob -= torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob

    def _soft_update(self):
        """Soft update target networks"""
        for param, target_param in zip(self.critic1.parameters(), self.critic1_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        for param, target_param in zip(self.critic2.parameters(), self.critic2_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
```

---

## Curriculum Learning

### Automatic Curriculum Design

Start with easy tasks, gradually increase difficulty:

```python
# curriculum_learning.py
import numpy as np

class CurriculumManager:
    def __init__(self, initial_difficulty=0.1, max_difficulty=1.0):
        self.difficulty = initial_difficulty
        self.max_difficulty = max_difficulty
        self.success_rate = 0.0
        self.success_window = []

    def update(self, success: bool):
        """Update difficulty based on recent success rate"""
        self.success_window.append(float(success))

        if len(self.success_window) > 100:
            self.success_window.pop(0)

        self.success_rate = np.mean(self.success_window)

        # Increase difficulty if success rate > 80%
        if self.success_rate > 0.8:
            self.difficulty = min(self.difficulty * 1.05, self.max_difficulty)

        # Decrease difficulty if success rate < 40%
        elif self.success_rate < 0.4:
            self.difficulty = max(self.difficulty * 0.95, 0.1)

    def sample_task(self):
        """Sample task based on current difficulty"""
        if self.difficulty < 0.3:
            # Easy: Large objects, close positions
            object_size = np.random.uniform(0.08, 0.12)
            target_distance = np.random.uniform(0.2, 0.4)

        elif self.difficulty < 0.7:
            # Medium: Smaller objects, farther positions
            object_size = np.random.uniform(0.05, 0.08)
            target_distance = np.random.uniform(0.4, 0.7)

        else:
            # Hard: Tiny objects, occluded, far
            object_size = np.random.uniform(0.02, 0.05)
            target_distance = np.random.uniform(0.7, 1.0)

        return {
            "object_size": object_size,
            "target_distance": target_distance,
            "difficulty": self.difficulty
        }

# Usage in training loop
curriculum = CurriculumManager()

for episode in range(10000):
    task_params = curriculum.sample_task()

    # Setup environment with task parameters
    env.reset(task_params)

    done = False
    while not done:
        action = agent.select_action(obs)
        obs, reward, done, info = env.step(action)

    # Update curriculum
    curriculum.update(success=info["success"])

    if episode % 100 == 0:
        print(f"Episode {episode}: Difficulty={curriculum.difficulty:.2f}, Success Rate={curriculum.success_rate:.2f}")
```

---

## Sim-to-Real Transfer

### Domain Randomization

**Randomize simulation parameters** to create robust policies:

```python
# domain_randomization.py
from omni.isaac.core import World
import numpy as np

class DomainRandomizer:
    def __init__(self, world: World):
        self.world = world

    def randomize_physics(self):
        """Randomize physics parameters"""
        # Randomize gravity
        gravity = np.random.uniform(9.5, 10.0)
        self.world.set_gravity([0, 0, -gravity])

        # Randomize mass
        for obj in self.world.scene.get_objects():
            mass_scale = np.random.uniform(0.8, 1.2)
            obj.set_mass(obj.get_mass() * mass_scale)

        # Randomize friction
        friction = np.random.uniform(0.3, 0.9)
        self.world.scene.get_ground_plane().set_friction(friction)

    def randomize_observations(self, obs):
        """Add noise to observations (sensor noise)"""
        # Position noise
        obs["position"] += np.random.normal(0, 0.002, size=obs["position"].shape)

        # Velocity noise
        obs["velocity"] += np.random.normal(0, 0.01, size=obs["velocity"].shape)

        # Force-torque noise
        if "force" in obs:
            obs["force"] += np.random.normal(0, 0.5, size=obs["force"].shape)

        return obs

    def randomize_actions(self, actions):
        """Add actuator delay and noise"""
        # Actuator noise
        actions += np.random.normal(0, 0.02, size=actions.shape)

        # Action clipping (actuator limits)
        actions = np.clip(actions, -1.0, 1.0)

        return actions

    def randomize_visuals(self):
        """Randomize visual appearance"""
        # Random textures (for vision-based policies)
        for obj in self.world.scene.get_objects():
            color = np.random.uniform(0, 1, size=3)
            obj.set_color(color)

        # Random lighting
        light_intensity = np.random.uniform(500, 2000)
        self.world.set_light_intensity(light_intensity)

# Training loop with randomization
randomizer = DomainRandomizer(world)

for episode in range(10000):
    # Randomize at episode start
    randomizer.randomize_physics()
    randomizer.randomize_visuals()

    obs = env.reset()

    done = False
    while not done:
        # Add observation noise
        noisy_obs = randomizer.randomize_observations(obs)

        action = agent.select_action(noisy_obs)

        # Add actuator noise
        noisy_action = randomizer.randomize_actions(action)

        obs, reward, done, info = env.step(noisy_action)
```

### System Identification

**Measure real robot parameters** and match simulation:

```python
# system_identification.py
import numpy as np
from scipy.optimize import minimize

class SystemIdentifier:
    def __init__(self, robot):
        self.robot = robot

    def identify_mass(self, joint_idx):
        """Estimate link mass from gravity compensation experiments"""
        # Move joint to different angles, measure torques
        angles = np.linspace(-1.0, 1.0, 20)
        measured_torques = []

        for angle in angles:
            self.robot.set_joint_position(joint_idx, angle)
            torque = self.robot.get_joint_torque(joint_idx)
            measured_torques.append(torque)

        # Fit model: torque = mass * g * length * sin(angle)
        def model(params, angle):
            mass, length = params
            return mass * 9.81 * length * np.sin(angle)

        def loss(params):
            predicted = [model(params, a) for a in angles]
            return np.sum((np.array(predicted) - np.array(measured_torques))**2)

        result = minimize(loss, x0=[1.0, 0.3], bounds=[(0.1, 10), (0.1, 1.0)])
        estimated_mass, estimated_length = result.x

        return {"mass": estimated_mass, "length": estimated_length}

    def identify_friction(self, joint_idx):
        """Estimate joint friction from velocity experiments"""
        # Move at constant velocities, measure required torques
        velocities = np.linspace(-2.0, 2.0, 20)
        measured_torques = []

        for vel in velocities:
            self.robot.set_joint_velocity(joint_idx, vel)
            torque = self.robot.get_joint_torque(joint_idx)
            measured_torques.append(torque)

        # Fit model: torque = viscous * vel + coulomb * sign(vel)
        def model(params, vel):
            viscous, coulomb = params
            return viscous * vel + coulomb * np.sign(vel)

        def loss(params):
            predicted = [model(params, v) for v in velocities]
            return np.sum((np.array(predicted) - np.array(measured_torques))**2)

        result = minimize(loss, x0=[0.1, 0.5], bounds=[(0, 2), (0, 5)])
        viscous_friction, coulomb_friction = result.x

        return {"viscous": viscous_friction, "coulomb": coulomb_friction}

# Usage: Tune simulation to match real robot
identifier = SystemIdentifier(real_robot)

for joint_idx in range(7):
    mass_params = identifier.identify_mass(joint_idx)
    friction_params = identifier.identify_friction(joint_idx)

    # Update simulation
    sim_robot.set_link_mass(joint_idx, mass_params["mass"])
    sim_robot.set_joint_friction(joint_idx, friction_params["viscous"], friction_params["coulomb"])

print("Simulation parameters updated to match real robot")
```

---

## Deployment on Physical Robots

### Safety-Critical RL

**Add safety constraints** before deploying:

```python
# safe_deployment.py
import torch

class SafetyFilter:
    def __init__(self, robot):
        self.robot = robot
        self.joint_limits = robot.get_joint_limits()
        self.velocity_limits = robot.get_velocity_limits()
        self.workspace_bounds = {"x": [-0.5, 0.5], "y": [-0.5, 0.5], "z": [0, 1.0]}

    def filter_action(self, action):
        """Apply safety checks to action"""
        # 1. Joint limit check
        current_position = self.robot.get_joint_positions()
        predicted_position = current_position + action * 0.01  # Assuming 100Hz control

        for i, (lower, upper) in enumerate(self.joint_limits):
            if predicted_position[i] < lower or predicted_position[i] > upper:
                action[i] = 0  # Stop motion toward limit

        # 2. Velocity limit check
        action = torch.clamp(action, -self.velocity_limits, self.velocity_limits)

        # 3. Workspace boundary check
        ee_pos = self.robot.get_end_effector_position()
        if not self._in_workspace(ee_pos):
            # Compute direction to workspace center
            center = torch.tensor([0, 0, 0.5])
            action = (center - ee_pos) * 0.1  # Move toward center

        return action

    def _in_workspace(self, position):
        """Check if position is within safe workspace"""
        x, y, z = position
        return (self.workspace_bounds["x"][0] <= x <= self.workspace_bounds["x"][1] and
                self.workspace_bounds["y"][0] <= y <= self.workspace_bounds["y"][1] and
                self.workspace_bounds["z"][0] <= z <= self.workspace_bounds["z"][1])

# Deploy policy with safety filter
safety_filter = SafetyFilter(robot)

obs = env.reset()
for step in range(1000):
    # Policy selects action
    action = policy.select_action(obs)

    # Safety filter modifies action
    safe_action = safety_filter.filter_action(action)

    # Execute safe action
    obs, reward, done, info = env.step(safe_action)
```

---

## Multi-Agent RL

### Collaborative Manipulation

Train multiple robots to collaborate:

```python
# multi_agent_manipulation.py
from omni.isaac.gym.vec_env import VecEnvBase
import torch

class MultiAgentEnv(VecEnvBase):
    def __init__(self, num_envs=1024, num_agents=2):
        super().__init__(num_envs=num_envs)
        self.num_agents = num_agents

        # Add multiple robots per environment
        for env_idx in range(num_envs):
            for agent_idx in range(num_agents):
                robot_path = f"/World/Env_{env_idx}/Robot_{agent_idx}"
                self.add_robot(robot_path, position=[agent_idx * 0.5, 0, 0])

    def step(self, actions):
        """
        Actions: (num_envs, num_agents, action_dim)
        """
        # Apply actions for all agents
        for agent_idx in range(self.num_agents):
            agent_actions = actions[:, agent_idx, :]
            self.robots[agent_idx].apply_actions(agent_actions)

        self.world.step()

        # Compute observations and rewards
        obs = self.get_observations()  # Shape: (num_envs, num_agents, obs_dim)
        rewards = self.compute_rewards()  # Shape: (num_envs, num_agents)

        return obs, rewards, self.check_done(), {}

    def compute_rewards(self):
        """Compute collaborative reward"""
        # Shared reward: both agents must grasp the object
        object_pos = self.get_object_position()

        agent_0_dist = torch.norm(self.robots[0].get_ee_position() - object_pos, dim=-1)
        agent_1_dist = torch.norm(self.robots[1].get_ee_position() - object_pos, dim=-1)

        # Reward when both agents are close to object
        collaborative_reward = -(agent_0_dist + agent_1_dist)

        # Bonus if object is lifted
        object_height = object_pos[:, 2]
        lift_bonus = torch.clamp(object_height - 0.3, 0, 1) * 10

        return collaborative_reward + lift_bonus
```

---

## Key Takeaways

- **Advanced RL algorithms** (HER, SAC) improve sample efficiency and enable sparse-reward learning
- **Curriculum learning** automatically adjusts task difficulty to accelerate training
- **Domain randomization** creates robust policies that transfer to real robots despite sim-to-real gap
- **System identification** tunes simulation parameters to match real robot dynamics
- **Safety filters** ensure deployed policies respect joint limits, workspace boundaries, and velocity constraints
- **Multi-agent RL** enables collaborative manipulation with coordinated policies

Mastering these techniques transforms RL from a research tool into a **production-ready** technology for deploying intelligent behaviors on physical robots.

---

**Previous Chapter**: [AI-Powered Perception](./ai-powered-perception.md)
**Next Module**: [Module 4: Vision-Language-Action Models](/docs/module-4-vla/intro-vla)

Congratulations! You've completed Module 3: NVIDIA Isaac. You now have the skills to simulate, train, and deploy intelligent robotic systems. In Module 4, we'll explore cutting-edge Vision-Language-Action models that enable robots to understand natural language instructions and visual scenes.
