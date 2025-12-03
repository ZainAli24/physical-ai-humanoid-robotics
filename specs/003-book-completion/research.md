# Research Findings: Book Completion - Modules 2-4, Search & Deployment

**Feature**: Book Completion - Modules 2-4, Search & Deployment
**Branch**: `003-book-completion`
**Created**: 2025-11-30
**Purpose**: Consolidate research findings to inform content creation for Modules 2-4

---

## R1: Gazebo Simulation Best Practices

**Research Question**: What are the recommended approaches for integrating Gazebo with ROS 2 Humble?

### Decision: Use Gazebo Garden (latest stable) with ROS 2 Humble integration

**Rationale**:
- Gazebo Garden is the officially recommended version for ROS 2 Humble (confirmed compatibility)
- Improved physics engine (ODE with better performance, optional Bullet support)
- Modern launch file patterns using Python launch API (consistent with Module 1)
- Better sensor simulation (GPU-accelerated ray tracing for LiDAR/cameras)
- Active development and community support

**Alternatives Considered**:
1. **Gazebo Fortress**: Older LTS release, stable but missing Garden's performance improvements and features
2. **Gazebo Classic (v11)**: Deprecated for ROS 2, not recommended for new projects (legacy ROS 1 focus)
3. **Ignition Gazebo**: Rebranded as "Gazebo" (Garden), so this IS the recommended choice

### Integration Patterns

**ROS 2 ↔ Gazebo Communication**:
- Use `ros_gz_bridge` package for bidirectional ROS 2 ↔ Gazebo topic/service bridging
- Gazebo publishes sensor data (camera images, LiDAR scans, IMU) to ROS 2 topics
- ROS 2 sends control commands (velocity, position) to Gazebo

**Launch File Pattern** (Python-based, consistent with Module 1):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),

        # Spawn robot model from URDF
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', urdf_file, '-name', 'robot'],
            output='screen'
        ),

        # Bridge ROS 2 topics ↔ Gazebo topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        )
    ])
```

**Model Organization**:
- URDF files in `<package>/urdf/` for robot descriptions (standard ROS 2 convention)
- SDF files in `<package>/worlds/` for Gazebo-specific world definitions
- Meshes in `<package>/meshes/` for visual/collision geometry
- Launch files in `<package>/launch/` for Python-based launch scripts

**Physics Engine Configuration**:
- Default: ODE (Open Dynamics Engine) - good for most robotics applications
- Alternative: Bullet - better for complex contact scenarios (grasping, manipulation)
- Configuration in SDF world file or via launch arguments

### Installation Steps (Ubuntu 22.04)

```bash
# Add Gazebo Garden repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Garden
sudo apt-get update
sudo apt-get install gz-garden

# Install ROS 2 Humble ↔ Gazebo bridge
sudo apt-get install ros-humble-ros-gz

# Verify installation
gz sim --version  # Should show Gazebo Garden version
```

### Chapter Content Recommendations

**Chapter 1: Introduction to Gazebo**:
- Focus on Gazebo Garden (not older versions)
- Emphasize ROS 2 Humble integration via `ros_gz_bridge`
- Include Python launch file examples (not XML)
- Demonstrate sensor simulation (camera, LiDAR, IMU)
- Hands-on: Spawn TurtleBot3 in Gazebo, control via ROS 2 `/cmd_vel`

**Key Concepts to Cover**:
1. Gazebo architecture: Simulation server + GUI client (can run headless)
2. SDF (Simulation Description Format) vs. URDF (robot-only)
3. Physics engines: ODE (default), Bullet (optional)
4. Sensor plugins: GPU-accelerated ray tracing for realistic LiDAR/cameras
5. ROS 2 integration: `ros_gz_bridge` for seamless communication

---

## R2: Unity Robotics Hub Setup

**Research Question**: What is the current best practice for Unity + ROS 2 integration?

### Decision: Use Unity 2022 LTS with Unity Robotics Hub and ROS TCP Connector

**Rationale**:
- Unity 2022 LTS (Long-Term Support) provides stability for educational content (supported until 2025)
- Unity Robotics Hub is the official Unity package for ROS/ROS 2 integration
- ROS TCP Connector enables reliable Unity ↔ ROS 2 communication over TCP/IP
- URDF Importer simplifies importing robot models from ROS packages into Unity
- Active community support and well-documented workflows

**Alternatives Considered**:
1. **Unity 2023 (latest)**: Not LTS, frequent breaking changes, less stable for educational use
2. **Direct ROS 2 native integration**: Not officially supported by Unity, requires custom middleware
3. **Unity 2021 LTS**: Older version, missing some Robotics Hub features (but still functional)

### Integration Architecture

**Unity ↔ ROS 2 Communication Flow**:
```
Unity (C# scripts)
    ↕ (Unity Robotics Hub API)
ROS TCP Endpoint (Unity side)
    ↕ (TCP/IP socket)
ROS TCP Endpoint (ROS 2 side - Python node)
    ↕ (ROS 2 topics/services)
ROS 2 Ecosystem
```

**Key Components**:
1. **Unity Robotics Hub Package**: Core package providing ROS/ROS 2 message support in Unity
2. **ROS TCP Connector**: Handles TCP/IP communication between Unity and ROS 2
3. **URDF Importer**: Imports URDF robot descriptions into Unity (ArticulationBody physics)
4. **Visualizations Package**: Tools for visualizing ROS topics in Unity (point clouds, markers)

### Installation Steps

**Unity Side (Unity 2022 LTS)**:
```
1. Install Unity Hub from https://unity.com/download
2. Install Unity 2022 LTS (e.g., Unity 2022.3.x LTS) via Unity Hub
3. Create new 3D project (or open existing)
4. Open Package Manager (Window → Package Manager)
5. Click "+" → "Add package from git URL"
6. Add Unity Robotics Hub:
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
7. Add URDF Importer:
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

**ROS 2 Side (Ubuntu 22.04 + ROS 2 Humble)**:
```bash
# Clone ROS TCP Endpoint package
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build workspace
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint

# Source workspace
source install/setup.bash

# Run ROS TCP Endpoint server (listens on port 10000 by default)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Unity C# Publisher Example

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        // Create Twist message for robot velocity
        TwistMsg twist = new TwistMsg
        {
            linear = new Vector3Msg { x = 0.5, y = 0, z = 0 },
            angular = new Vector3Msg { x = 0, y = 0, z = 0.2 }
        };

        // Publish to ROS 2
        ros.Publish(topicName, twist);
    }
}
```

### URDF Import Workflow

1. **Export URDF from ROS package**:
   ```bash
   # Assuming robot description in ~/ros2_ws/src/my_robot/urdf/robot.urdf
   # Copy to accessible location for Unity
   cp ~/ros2_ws/src/my_robot/urdf/robot.urdf ~/Downloads/
   ```

2. **Import into Unity**:
   - In Unity: Assets → Import Robot from URDF
   - Select `robot.urdf` file
   - Configure import options:
     - Axis type: Y-up (Unity convention)
     - Mesh decomposer: vHACD (for collision meshes)
     - Create ArticulationBody (Unity physics for robots)
   - Click "Import URDF"

3. **Result**: Robot model with ArticulationBody joints (Unity's advanced physics for robotics)

### Chapter Content Recommendations

**Chapter 2: Unity for Robotics**:
- Focus on Unity 2022 LTS (stable, long-term support)
- Emphasize ROS TCP Connector for ROS 2 communication
- Include C# code examples for publishing/subscribing to ROS 2 topics
- Demonstrate URDF import workflow (ROS URDF → Unity ArticulationBody)
- Hands-on: Import TurtleBot3 URDF, control from ROS 2, visualize in Unity

**Key Concepts to Cover**:
1. Unity Robotics Hub architecture (TCP/IP bridge)
2. ArticulationBody physics (Unity's joint-based physics for robots)
3. ROS message serialization in Unity (C# message types)
4. URDF import process (axis conventions, mesh decomposition)
5. Bidirectional communication: Unity → ROS 2 (sensor data) and ROS 2 → Unity (control commands)

---

## R3: NVIDIA Isaac Installation & Setup

**Research Question**: What are the system requirements and installation steps for Isaac Sim and Isaac Gym?

### Decision: Use Isaac Sim 2023.1.0 (latest stable) via Omniverse Launcher

**Rationale**:
- Isaac Sim 2023.1.0 is the latest stable release (November 2023)
- Omniverse Launcher provides easiest installation and update path
- Built-in ROS 2 bridge (native support for Humble)
- GPU-accelerated physics (PhysX 5) and rendering (RTX ray tracing)
- Isaac Gym functionality integrated into Isaac Sim (no separate installation needed)

**Alternatives Considered**:
1. **Standalone Isaac Sim**: Deprecated, Omniverse Launcher is now the official method
2. **Isaac Gym (standalone)**: Legacy, now integrated into Isaac Sim as "Isaac Gym Preview"
3. **Docker containers**: Available but adds complexity for beginners (use Omniverse Launcher for education)

### System Requirements

**Minimum Requirements**:
- **GPU**: NVIDIA RTX 2060 or higher (6GB VRAM minimum)
- **CUDA**: 11.8 or higher (installed automatically with Isaac Sim)
- **Driver**: NVIDIA driver 525.x or higher
- **OS**: Ubuntu 22.04 LTS (official support)
- **RAM**: 16GB minimum (32GB recommended for complex scenes)
- **Disk Space**: 50GB for Omniverse and Isaac Sim

**Recommended Requirements**:
- **GPU**: NVIDIA RTX 3080 or RTX 4080 (12GB+ VRAM for large-scale RL training)
- **CPU**: Intel Core i7/i9 or AMD Ryzen 7/9 (8+ cores)
- **RAM**: 32GB or more
- **Storage**: NVMe SSD for faster asset loading

**Verify GPU Support**:
```bash
# Check NVIDIA GPU
lspci | grep -i nvidia

# Check CUDA support
nvidia-smi

# Expected output: RTX series GPU, driver 525+, CUDA 11.8+
```

### Installation Steps (Ubuntu 22.04)

**Step 1: Install NVIDIA Drivers (if not installed)**
```bash
# Add NVIDIA driver repository
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install latest driver (525+)
sudo apt install nvidia-driver-535  # Or latest version

# Reboot
sudo reboot

# Verify installation
nvidia-smi  # Should show GPU, driver version, CUDA version
```

**Step 2: Install Omniverse Launcher**
```bash
# Download Omniverse Launcher for Linux
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run Launcher
./omniverse-launcher-linux.AppImage
```

**Step 3: Install Isaac Sim via Omniverse Launcher**
1. Open Omniverse Launcher
2. Sign in with NVIDIA account (free registration)
3. Navigate to "Exchange" tab
4. Search for "Isaac Sim"
5. Click "Install" for Isaac Sim 2023.1.0
6. Wait for download and installation (~20GB)

**Step 4: Verify Isaac Sim Installation**
```bash
# Launch Isaac Sim from Omniverse Launcher (GUI)
# Or from command line:
~/.local/share/ov/pkg/isaac_sim-2023.1.0/isaac-sim.sh

# Expected: Isaac Sim GUI opens with sample scenes
```

### ROS 2 Integration (Isaac ROS)

Isaac Sim includes native ROS 2 support via Isaac ROS packages:

```bash
# Isaac ROS packages (optional, for advanced ROS 2 integration)
sudo apt-get install ros-humble-isaac-ros-*

# Isaac Sim has built-in ROS 2 bridge (no extra install needed)
# Enable in Isaac Sim: Window → Extensions → ROS2 Bridge
```

**Publishing ROS 2 Topics from Isaac Sim**:
- Camera images: `/camera/rgb/image_raw`, `/camera/depth/image_raw`
- LiDAR scans: `/scan`, `/point_cloud`
- IMU data: `/imu/data`
- Joint states: `/joint_states`
- Odometry: `/odom`

### Isaac Gym (Reinforcement Learning)

**Note**: Isaac Gym is now integrated into Isaac Sim as "Isaac Gym Preview"

**Accessing Isaac Gym in Isaac Sim**:
1. Open Isaac Sim
2. Navigate to: Isaac Examples → Isaac Gym → Cartpole (example RL environment)
3. Run RL training: Uses GPU-accelerated parallel environments

**Python API for RL Training** (IsaacGymEnvs):
```bash
# Clone IsaacGymEnvs repository
git clone https://github.com/NVIDIA-Omniverse/IsaacGymEnvs.git
cd IsaacGymEnvs

# Install dependencies
pip install -e .

# Train Cartpole example with PPO
python train.py task=Cartpole

# Expected: Parallel environments training on GPU, real-time visualization
```

### Chapter Content Recommendations

**Chapter 1: Introduction to NVIDIA Isaac**:
- System requirements (emphasize RTX GPU requirement)
- Installation via Omniverse Launcher (step-by-step)
- Isaac Sim vs. Isaac Gym (Gym is now integrated into Sim)
- Overview of USD (Universal Scene Description) format
- ROS 2 Humble integration (built-in bridge)

**Chapter 2: Isaac Sim for Robotics**:
- Creating simulation scenes (USD stage, prims)
- Importing robot models (URDF support)
- Adding sensors (camera, LiDAR, IMU)
- ROS 2 bridge configuration (publishing sensor data)
- Hands-on: Load robot, add camera, publish `/camera/rgb/image_raw` to ROS 2

**Chapter 3: Reinforcement Learning with Isaac Gym**:
- Isaac Gym Preview in Isaac Sim
- Parallel environments (1000+ simultaneous simulations on single GPU)
- RL algorithms: PPO (Proximal Policy Optimization), SAC (Soft Actor-Critic)
- IsaacGymEnvs framework (Python API for custom RL tasks)
- Hands-on: Train Cartpole, visualize policy learning, export trained model

**Key Concepts to Cover**:
1. GPU-accelerated physics (PhysX 5) vs. CPU-based (Gazebo)
2. USD format for scene description (Pixar's open standard)
3. Tensor-based simulation (vectorized physics for RL)
4. Synthetic data generation (domain randomization for sim-to-real)
5. ROS 2 integration for real robot deployment

---

## R4: VLA Model State-of-the-Art

**Research Question**: What are the current production-ready VLA models and their architectures?

### Decision: Focus on RT-1, RT-2, and PaLM-E as exemplary VLA models

**Rationale**:
- RT-1 (2022): First successful large-scale VLA model from Google Robotics
- RT-2 (2023): Improved version leveraging vision-language model pretraining
- PaLM-E (2023): Multimodal embodied language model with broader capabilities
- All three models have published papers, documented architectures, and real-world deployments
- Represent the evolution of VLA: RT-1 (specialist) → RT-2 (generalist) → PaLM-E (multimodal reasoner)

**Alternatives Considered**:
1. **CLIP-based models**: Vision-language models but not action-conditioned (no robotics focus)
2. **Flamingo**: Multimodal model but not embodied (no action decoder)
3. **Gato (DeepMind)**: Generalist agent but architecture less transparent than RT-1/RT-2

### VLA Model Comparison

| Model | Organization | Release | Parameters | Training Data | Key Innovation |
|-------|--------------|---------|------------|---------------|----------------|
| **RT-1** | Google Robotics | 2022 | 35M | 130K robot demos | Token Learner + Transformer for actions |
| **RT-2** | Google DeepMind | 2023 | 55B (PaLI-X backbone) | Web-scale vision-language + robot demos | Vision-language model fine-tuned for actions |
| **PaLM-E** | Google Research | 2023 | 562B (largest variant) | Text + vision + robot data | Multimodal embodied reasoning |

### RT-1 (Robotics Transformer 1) Architecture

**Paper**: "RT-1: Robotics Transformer for Real-World Control at Scale" (2022)

**Architecture Overview**:
```
Input: Image (6Hz camera feed) + Language Instruction (text)
    ↓
[Token Learner] → Compress image to visual tokens
    ↓
[Transformer Encoder-Decoder] → Process visual + language tokens
    ↓
[Action Token Decoder] → Generate discrete action tokens
    ↓
Output: 7-DOF robot arm actions (x, y, z, roll, pitch, yaw, gripper)
```

**Key Components**:
1. **Token Learner**: Compresses 300×300 RGB image to 81 visual tokens (efficient representation)
2. **Transformer**: 8-layer encoder + 8-layer decoder (35M parameters total)
3. **Action Tokenization**: Discretizes continuous actions into 256 bins per dimension
4. **Training Data**: 130,000 demonstrations from 13 robots over 17 months

**Training Approach**:
- Supervised learning on expert demonstrations (teleoperation)
- Task families: Pick-and-place, drawer opening, object manipulation
- Generalization: 700+ tasks with 97% success on seen tasks, 76% on novel tasks

**Inference**:
- Real-time: 3Hz action prediction (333ms per action)
- Input: Current image + text instruction (e.g., "pick up the can")
- Output: Next robot action (joint velocities + gripper command)

### RT-2 (Robotics Transformer 2) Architecture

**Paper**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (2023)

**Architecture Overview**:
```
Input: Image + Language Instruction
    ↓
[PaLI-X Vision-Language Model] → Pretrained on web-scale data
    ↓
[Fine-tuning] → Add robot action tokens to vocabulary
    ↓
[Action Decoder] → Generate robot actions as text tokens
    ↓
Output: 7-DOF robot arm actions (same as RT-1)
```

**Key Innovation: Web-Scale Pretraining**
- **Base Model**: PaLI-X (55B parameters, vision-language model trained on web data)
- **Fine-tuning**: Add robot action tokens to PaLI-X vocabulary (e.g., "action: move left 10cm")
- **Result**: Model can reason about novel objects and tasks using web knowledge

**Advantages over RT-1**:
1. **Emergent Capabilities**: Can handle objects never seen in robot training data (e.g., "pick up the extinct animal" → correctly identifies toy dinosaur)
2. **Symbol Understanding**: Understands visual symbols (e.g., "move to the Taylor Swift fan" → moves to object with "1989" text)
3. **Reasoning**: Can perform multi-step reasoning tasks (e.g., "pick up the bag of chips for someone at the table")

**Training Data**:
- Pretraining: Web images + captions (billions of examples)
- Fine-tuning: 130K robot demonstrations (same as RT-1)

**Performance**:
- 62% success on unseen objects (vs. 32% for RT-1)
- 3× better on reasoning tasks requiring visual understanding

### PaLM-E (Embodied Language Model) Architecture

**Paper**: "PaLM-E: An Embodied Multimodal Language Model" (2023)

**Architecture Overview**:
```
Input: Text + Images + Sensor Data (optional: robot state, depth)
    ↓
[Input Encoder] → Encode images and sensor data as embeddings
    ↓
[PaLM Language Model] → 540B parameter language model
    ↓
[Multimodal Fusion] → Interleave text tokens + vision tokens + sensor tokens
    ↓
[Decoder] → Generate text (planning) OR actions (robot control)
    ↓
Output: Natural language plan OR robot actions
```

**Key Innovations**:
1. **Multimodal Integration**: Combines text, vision, and proprioceptive data in single model
2. **Task Generalization**: Can perform language tasks (QA, reasoning) AND robotics tasks
3. **Planning Capabilities**: Generates step-by-step plans (e.g., "Task: make breakfast" → steps: "1. Go to fridge. 2. Open door. 3. Take eggs...")

**Training Approach**:
- **Transfer Learning**: Pretrained PaLM-540B language model
- **Vision Encoder**: ViT (Vision Transformer) for image understanding
- **Robot Data**: 10,000+ robot demonstrations across multiple robots

**Capabilities**:
1. **Embodied Reasoning**: "Where is the nearest recycling bin?" → Understands spatial relations
2. **Affordance Understanding**: "Can you open this?" → Identifies openable objects (doors, drawers)
3. **Long-Horizon Planning**: "Clean the table" → Plan: 1. Grasp object. 2. Move to trash. 3. Repeat.

**Performance**:
- 90%+ success on multi-step mobile manipulation tasks
- Maintains language understanding (100% on OK-VQA benchmark while solving robot tasks)

### Deployment Considerations

**Model Size vs. Inference Speed**:
| Model | Parameters | Inference Time | Hardware Requirement |
|-------|------------|----------------|----------------------|
| RT-1 | 35M | 333ms/action | GPU (RTX 3060+) |
| RT-2 | 55B | ~1s/action | GPU cluster (A100 recommended) |
| PaLM-E | 562B | ~5s/action | Multi-GPU cluster (A100×4+) |

**Practical Recommendations**:
- **RT-1**: Suitable for real-time robot control (3Hz) on consumer GPUs
- **RT-2**: Requires datacenter GPUs (A100) for real-time operation
- **PaLM-E**: Research use only (too slow for real-time control without distillation)

**Distillation Strategy**:
- Train small "student" model (RT-1 size) to mimic large "teacher" model (RT-2/PaLM-E)
- Result: Real-time inference with improved capabilities from teacher model

### Chapter Content Recommendations

**Chapter 1: Introduction to VLA Models**:
- Definition: Vision-Language-Action = multimodal models for robotics
- Evolution: Traditional RL → Imitation Learning → VLA (language-conditioned)
- Use cases: Household robots, warehouse automation, assistive robotics
- Advantages: Natural language control, generalization to novel objects

**Chapter 2: VLA Architectures (RT-1, RT-2, PaLM-E)**:
- RT-1: Token Learner + Transformer (specialist, fast inference)
- RT-2: Vision-language model fine-tuning (generalist, web knowledge transfer)
- PaLM-E: Embodied language model (multimodal reasoning, planning)
- Comparison table (parameters, data, capabilities, inference speed)
- Architecture diagrams (Mermaid flowcharts for each model)

**Chapter 3: Training and Deploying VLA Systems**:
- Data collection: Teleoperation, kinesthetic teaching, simulation
- Training pipeline: Pretraining (web data) → Fine-tuning (robot demos)
- Deployment: Model distillation for real-time inference
- Safety considerations: Failure recovery, human oversight
- Case study: Google's Everyday Robots using RT-2 in office environments

**Key Concepts to Cover**:
1. Action tokenization (discretizing continuous actions for transformer training)
2. Vision-language pretraining (leveraging web data for robotics)
3. Emergent capabilities (zero-shot generalization to novel objects/tasks)
4. Multimodal fusion (combining vision, language, proprioception)
5. Sim-to-real transfer (using Isaac Gym simulations to augment real-world data)

---

## R5: Search Plugin Selection

**Research Question**: Should we use local search plugin or Algolia DocSearch for this textbook?

### Decision: Use @easyops-cn/docusaurus-search-local (local search plugin)

**Rationale**:
- **Site Size**: 12 total chapters (Module 1: 3 chapters + Modules 2-4: 9 chapters) = well under 100-page limit for local search
- **Offline Capability**: Local search works without internet connection (important for students with limited connectivity)
- **Zero Cost**: Free, no API keys or external dependencies
- **Privacy**: No search queries sent to external servers
- **Easy Setup**: Single npm install, works immediately without approval process

**Alternatives Considered**:

| Option | Pros | Cons | Recommendation |
|--------|------|------|----------------|
| **@easyops-cn/docusaurus-search-local** | Free, offline, easy setup, privacy | Limited to ~100 pages | ✅ **CHOOSE THIS** |
| **Algolia DocSearch** | Fast, scalable, advanced features | Requires approval, external dependency, online-only | ❌ Not needed for 12 chapters |
| **Typesense** | Self-hosted, fast | Requires server setup, complexity | ❌ Overkill for static site |

### Implementation Details

**Installation**:
```bash
npm install --save @easyops-cn/docusaurus-search-local
```

**Configuration** (docusaurus.config.js):
```javascript
module.exports = {
  // ... other config
  themes: [
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        // Index all docs pages
        hashed: true,  // Hash index files for cache busting
        language: ["en"],  // English only
        indexDocs: true,  // Index all documentation pages
        indexBlog: false,  // No blog content in this textbook
        indexPages: false,  // Don't index generic pages (only docs)

        // Search configuration
        docsRouteBasePath: '/docs',  // Base path for documentation
        highlightSearchTermsOnTargetPage: true,  // Highlight keywords on result page
        searchResultLimits: 8,  // Show max 8 results
        searchResultContextMaxLength: 50,  // Context snippet length

        // Appearance
        explicitSearchResultPath: true,  // Show full path in results
      },
    ],
  ],
  // ... other config
};
```

**Build Process**:
```bash
# During build, search index is generated automatically
npm run build

# Index files created in build/ directory:
# build/search-index.json (or hashed filename if hashed: true)
```

**Testing**:
```bash
# Start dev server
npm start

# Visit http://localhost:3000
# Search bar appears in navigation header
# Try searches: "ROS 2", "Gazebo", "Isaac Sim", "VLA"
```

### Search Features

**Keyword Highlighting**:
- Search results highlight matching keywords in yellow
- When clicking result, keywords highlighted on target page

**Search Scope**:
- Indexes all heading text (H1, H2, H3, H4)
- Indexes all paragraph text
- Indexes code block comments (if enabled)
- Indexes image alt text (for accessibility)

**Search Syntax**:
- **Basic search**: `ROS 2 topics` → Finds pages containing "ROS", "2", "topics"
- **Phrase search**: `"ros2 topic pub"` → Exact phrase match (with quotes)
- **Exclusion**: `Gazebo -Unity` → Pages with "Gazebo" but not "Unity"

### Performance

**Index Size** (estimated for 12 chapters):
- 12 chapters × ~2000 words each = ~24,000 words
- Search index JSON file: ~200-300KB (compressed)
- Load time: <100ms on modern browsers

**Search Speed**:
- Client-side search (JavaScript)
- Results appear as user types (instant)
- No network latency (all data local)

### Accessibility

**Keyboard Navigation**:
- Press `/` to focus search bar
- Arrow keys to navigate results
- Enter to open selected result
- Escape to close search

**Screen Reader Support**:
- Search bar has proper ARIA labels
- Results announced to screen readers
- Keyboard-only navigation supported

### Chapter Content Recommendations

**Search Usage Documentation** (in deployment chapter or README):
- How to use search bar (keyboard shortcuts, syntax)
- Search tips for students (quotation marks for exact match, exclusion with `-`)
- Offline capability (search works without internet)

---

## R6: GitHub Pages Deployment Workflow

**Research Question**: What is the recommended GitHub Actions workflow for Docusaurus deployment to GitHub Pages?

### Decision: Use GitHub Actions with official `actions/deploy-pages` action

**Rationale**:
- **Official Support**: GitHub provides `actions/deploy-pages@v4` action (maintained by GitHub)
- **Automatic HTTPS**: GitHub Pages provides free HTTPS for all sites
- **Zero Cost**: Free hosting for public repositories
- **CI/CD Integration**: Automatic deployment on push to main branch
- **Rollback Safety**: Failed builds don't deploy (preserves last working version)

**Alternatives Considered**:
1. **Vercel**: Fast, edge network, but requires separate account (GitHub Pages simpler)
2. **Netlify**: Similar to Vercel, extra setup not needed for static docs
3. **Manual `gh-pages` branch**: Deprecated approach, GitHub Actions is now recommended

### GitHub Actions Workflow Structure

**File**: `.github/workflows/deploy.yml`

```yaml
name: Deploy to GitHub Pages

on:
  # Trigger on push to main branch
  push:
    branches:
      - main
  # Allow manual triggering from Actions tab
  workflow_dispatch:

# Required permissions for GitHub Pages deployment
permissions:
  contents: read   # Read repository contents
  pages: write     # Write to GitHub Pages
  id-token: write  # Verify deployment identity

# Ensure only one deployment runs at a time
concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  # Build job: Compile Docusaurus site
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm  # Cache node_modules for faster builds

      - name: Install dependencies
        run: npm ci  # Clean install (faster than npm install)

      - name: Build site
        run: npm run build
        env:
          NODE_ENV: production

      - name: Upload artifact for deployment
        uses: actions/upload-pages-artifact@v3
        with:
          path: build  # Docusaurus build output directory

  # Deploy job: Publish to GitHub Pages
  deploy:
    needs: build  # Must wait for build job to complete
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

### Build Validation (Error Handling)

**Docusaurus Build Errors Caught**:
1. **Broken Links**: `npm run build` fails if internal links point to non-existent pages
2. **Missing Images**: Build fails if Markdown references non-existent image files
3. **Invalid Mermaid**: Build fails if Mermaid diagram syntax is incorrect
4. **Broken Frontmatter**: Build fails if YAML frontmatter has syntax errors

**Example Error Output**:
```bash
$ npm run build
[ERROR] Broken link on source page path=/docs/module-2/intro-gazebo:
-> linking to /docs/module-2/missing-page (resolved as: docs/module-2/missing-page.md)
[ERROR] Image not found: static/img/module-2/missing-diagram.png
```

**Automatic Rollback**:
- If build job fails → deploy job doesn't run (GitHub Actions dependency: `needs: build`)
- Previous successful deployment remains live on GitHub Pages
- GitHub sends email notification to repository owner on workflow failure

### Docusaurus Configuration for GitHub Pages

**File**: `docusaurus.config.js`

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to ROS 2, Simulation, and VLA',

  // IMPORTANT: Update these for your repository
  url: 'https://username.github.io',  // Your GitHub Pages URL
  baseUrl: '/physical-ai-humanoid-robotics/',  // Repository name

  organizationName: 'username',  // GitHub username or org
  projectName: 'physical-ai-humanoid-robotics',  // Repository name

  // GitHub Pages deployment config
  deploymentBranch: 'gh-pages',  // Branch where built site is deployed
  trailingSlash: false,  // URL format preference

  // ... rest of config
};
```

### Repository Settings Configuration

**Enable GitHub Pages**:
1. Go to repository Settings → Pages
2. Source: Select "GitHub Actions" (NOT "Deploy from a branch")
3. Custom domain (optional): Leave blank for default `username.github.io/repo-name`
4. Enforce HTTPS: Enabled (default)

**Branch Protection** (recommended):
- Protect `main` branch: Require pull request reviews before merging
- Require status checks: Ensure `build` job passes before allowing merge
- Prevents broken deployments from reaching production

### Deployment Timeline

**Expected Workflow Duration**:
```
Push to main
    ↓ (GitHub detects push, triggers workflow)
Build job starts (~30s)
    ↓ (Checkout, setup Node, install deps, build)
Build completes (~2-3 minutes for 12 chapters)
    ↓ (Upload build artifact)
Deploy job starts (~10s)
    ↓ (Download artifact, deploy to Pages)
Deploy completes (~30s)
    ↓ (GitHub Pages propagates changes)
Site live at https://username.github.io/repo-name (~5 minutes total)
```

**Monitoring Deployment**:
- GitHub Actions tab shows real-time workflow progress
- Green checkmark = successful deployment
- Red X = build failed (check logs for errors)
- Deployed site URL shown in deploy job output

### Optimization Strategies

**Cache Dependencies**:
```yaml
- name: Setup Node.js
  uses: actions/setup-node@v4
  with:
    node-version: 18
    cache: npm  # Caches node_modules between runs
```
**Benefit**: Reduces install time from 60s to ~10s on subsequent runs

**Parallel Builds** (for monorepos with multiple sites):
```yaml
strategy:
  matrix:
    site: [docs, api-reference]
```
**Benefit**: Build multiple Docusaurus sites simultaneously (not applicable to this single-site book)

### Chapter Content Recommendations

**Deployment Chapter Content**:
- GitHub Actions workflow structure (build → deploy)
- Build validation (broken links, missing images, Mermaid errors)
- Repository settings (enable GitHub Pages, select "GitHub Actions" source)
- Monitoring deployments (Actions tab, email notifications)
- Troubleshooting (common errors, rollback safety, checking logs)

**Hands-on Exercise**:
1. Fork repository to personal GitHub account
2. Enable GitHub Pages (Settings → Pages → GitHub Actions)
3. Push change to main branch (edit README.md)
4. Monitor workflow in Actions tab
5. Verify site deploys to `https://username.github.io/repo-name`

---

## Summary of Research Decisions

| Research Topic | Decision | Rationale |
|----------------|----------|-----------|
| **R1: Gazebo** | Gazebo Garden with ros_gz_bridge | Official ROS 2 Humble support, modern Python launch API |
| **R2: Unity** | Unity 2022 LTS + Robotics Hub | Stable LTS version, official ROS 2 integration via TCP |
| **R3: Isaac** | Isaac Sim 2023.1.0 via Omniverse | Latest stable, integrated Isaac Gym, built-in ROS 2 bridge |
| **R4: VLA** | RT-1, RT-2, PaLM-E comparison | Evolution of VLA models, published architectures, real deployments |
| **R5: Search** | Local search plugin (@easyops-cn) | Free, offline, sufficient for 12 chapters, easy setup |
| **R6: Deployment** | GitHub Actions + deploy-pages@v4 | Official GitHub support, automatic HTTPS, zero cost, rollback safety |

**All Research Complete**: Ready for T013 (Content Outline Creation)

---

**Status**: ✅ Research Phase Complete (T007-T012)
**Next Task**: T013 - Create detailed content outlines for all 9 chapters
**Created**: 2025-11-30
**Last Updated**: 2025-11-30
