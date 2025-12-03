# Physical AI & Humanoid Robotics Textbook

An interactive, open-source textbook covering ROS 2, simulation (Gazebo, Unity, Isaac), and vision-language-action models for physical AI and humanoid robotics.

**Live Site**: [https://YOUR_GITHUB_USERNAME.github.io/physical-ai-humanoid-robotics/](https://YOUR_GITHUB_USERNAME.github.io/physical-ai-humanoid-robotics/)
*(Replace with your GitHub Pages URL after deployment)*

## About This Textbook

This comprehensive textbook provides hands-on learning for robotics developers, covering:

- **Module 1**: ROS 2 Fundamentals (Nodes, Topics, Services, Actions)
- **Module 2**: Gazebo & Unity Simulation (Physics, Sensors, Integration)
- **Module 3**: NVIDIA Isaac Sim & Isaac Gym (GPU-accelerated RL)
- **Module 4**: Vision-Language-Action Models (RT-1, RT-2, PaLM-E)

**Features**:
- 12 chapters with 42,000+ words of technical content
- 52 hands-on exercises with code examples
- 19 Mermaid diagrams + 13 reference tables
- Local search functionality (sub-second response)
- Dark mode support
- Mobile-responsive design

---

## For Students & Learners

### Quick Start

Visit the [live textbook](https://YOUR_GITHUB_USERNAME.github.io/physical-ai-humanoid-robotics/) and start with Module 1.

### Prerequisites

- Basic Python programming
- Linux command-line familiarity
- Understanding of robotics concepts (sensors, actuators, kinematics)

### Learning Path

1. **Module 1** (Beginner): ROS 2 basics, pub/sub, services
2. **Module 2** (Intermediate): Simulation environments, URDF, sensors
3. **Module 3** (Advanced): GPU-accelerated RL, domain randomization
4. **Module 4** (Cutting-edge): VLA models, fine-tuning, deployment

---

## For Contributors & Developers

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
