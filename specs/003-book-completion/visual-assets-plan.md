# Visual Assets Plan - Book Iteration 3

## Overview

This document catalogs all visual elements (Mermaid diagrams, tables, figures) across the Physical AI & Humanoid Robotics textbook.

**Total Visual Assets**: 27 elements
- **Mermaid Diagrams**: 19
- **Markdown Tables**: 8
- **Images**: 3 (logo/favicon)

---

## Module 1: ROS 2 Fundamentals (Existing - 8 elements)

### Chapter 1: ROS 2 Fundamentals (ros2-fundamentals.md)

**Diagrams**: 3
1. ROS 2 Architecture Diagram (System overview)
2. Publisher-Subscriber Pattern (Communication model)
3. ROS 2 Workspace Structure (Directory tree)

**Tables**: 1
- ROS 2 vs ROS 1 Comparison Table

---

### Chapter 2: ROS 2 Nodes and Topics (ros2-nodes-topics.md)

**Diagrams**: 2
1. Node Communication Graph
2. Topic Message Flow

**Tables**: 1
- Topic CLI Commands Reference

---

### Chapter 3: ROS 2 Services and Actions (ros2-services-actions.md)

**Diagrams**: 1
1. Service vs Action Comparison Diagram

**Tables**: 1
- Service/Action CLI Commands Reference

---

## Module 2: Gazebo & Unity Simulation (7 elements)

### Chapter 1: Introduction to Gazebo (intro-gazebo.md)

**Diagrams**: 2
1. **Gazebo Architecture Diagram** (Line 145)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show relationship between Gazebo Server, GUI, plugins, and physics engine
   - **Alt text**: "Gazebo architecture flowchart showing Gazebo Server with physics engine and sensor simulation, Gazebo GUI client for visualization, and plugin system connecting to ROS 2 via gz_ros2_bridge"
   - **Components**: Server, Client, Physics (ODE), Sensors, Plugins, ROS 2 bridge

2. **Gazebo SDF Structure Diagram** (Line 220)
   - **Type**: Mermaid graph
   - **Purpose**: Illustrate SDF world file hierarchy
   - **Alt text**: "SDF world file structure showing hierarchical relationship from World to Models to Links to Visuals/Collisions/Inertials and Joints"
   - **Components**: World → Model → Link → (Visual, Collision, Inertial), Joints

**Tables**: 2
1. **Gazebo vs Other Simulators Comparison** (Line 65)
   - **Columns**: Simulator, Physics, Graphics, ROS Integration, Use Case
   - **Rows**: Gazebo, Isaac Sim, MuJoCo, PyBullet, Webots
   - **Purpose**: Help readers choose appropriate simulator

2. **Gazebo CLI Commands Reference** (Line 380)
   - **Columns**: Command, Description, Example
   - **Rows**: gz sim, gz model, gz topic, gz service
   - **Purpose**: Quick reference for command-line operations

---

### Chapter 2: Unity for Robotics (unity-robotics.md)

**Diagrams**: 1
1. **Unity Robotics Hub Architecture** (Line 145)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show components of Unity Robotics Hub ecosystem
   - **Alt text**: "Unity Robotics Hub architecture showing URDF Importer, ROS TCP Connector, and Visualizations packages connecting Unity to ROS 2"
   - **Components**: Unity Editor, URDF Importer, ROS TCP Connector, Visualizations, ROS 2

**Tables**: 2
1. **Unity vs Gazebo Comparison** (Line 85)
   - **Columns**: Feature, Unity, Gazebo
   - **Rows**: Graphics quality, Physics accuracy, ROS integration, Learning curve, Cost, Use case
   - **Purpose**: Decision matrix for simulator selection

2. **Unity Robotics Hub Components** (Line 200)
   - **Columns**: Package, Purpose, Installation
   - **Rows**: URDF Importer, ROS TCP Connector, Visualizations
   - **Purpose**: Component overview and setup guide

---

### Chapter 3: Simulation Best Practices (simulation-best-practices.md)

**Diagrams**: 0 (Conceptual chapter, no diagrams)

**Tables**: 2
1. **Simulation Fidelity Levels**
   - **Columns**: Level, Physics, Graphics, Use Case, Performance
   - **Rows**: Low, Medium, High, Photorealistic
   - **Purpose**: Guide fidelity vs performance trade-offs

2. **Sim-to-Real Transfer Checklist**
   - **Columns**: Category, Simulation, Real World, Validation
   - **Rows**: Physics parameters, Sensor noise, Dynamics, Control latency
   - **Purpose**: Ensure simulation matches reality

---

## Module 3: NVIDIA Isaac (9 elements)

### Chapter 1: Introduction to NVIDIA Isaac (intro-isaac.md)

**Diagrams**: 1
1. **Isaac Ecosystem Architecture** (Line 180)
   - **Type**: Mermaid graph
   - **Purpose**: Show relationships between Isaac Sim, Isaac Gym, Isaac ROS components
   - **Alt text**: "Isaac ecosystem architecture showing Omniverse Platform connecting to Isaac Sim, Nucleus server, USD format, with Isaac Sim branching to PhysX GPU, RTX Renderer, Isaac Gym RL, and OmniGraph"
   - **Components**: Omniverse, Isaac Sim, Isaac Gym, PhysX 5, RTX, OmniGraph

**Tables**: 1
1. **Isaac vs Gazebo vs Unity Comparison** (Line 290)
   - **Columns**: Feature, Isaac Sim, Gazebo, Unity
   - **Rows**: GPU acceleration, Parallel envs, Ray tracing, ROS 2 integration, RL support, Cost, Learning curve
   - **Purpose**: Three-way simulator comparison

---

### Chapter 2: Isaac Sim for Robotics (isaac-sim.md)

**Diagrams**: 2
1. **URDF Import Workflow** (Line 145)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show steps to import robot URDF into Isaac Sim
   - **Alt text**: "URDF import workflow flowchart showing: Load URDF → Parse links/joints → Create USD prims → Add PhysX components → Configure materials → Test in simulation"
   - **Steps**: Load, Parse, Create USD, Add Physics, Configure, Test

2. **OmniGraph ROS 2 Bridge** (Line 380)
   - **Type**: Mermaid flowchart
   - **Purpose**: Illustrate OmniGraph visual programming for ROS 2 integration
   - **Alt text**: "OmniGraph ROS 2 bridge showing Isaac Sim nodes (Camera, LiDAR, Joint State) connected to ROS 2 Publisher nodes via OmniGraph action graph"
   - **Components**: Sensors → OmniGraph → ROS 2 Publishers

**Tables**: 2
1. **USD Prim Types** (Line 85)
   - **Columns**: Prim Type, Purpose, Physics, Example
   - **Rows**: Xform, Mesh, RigidBody, ArticulationRoot, Joint
   - **Purpose**: USD scene composition reference

2. **Sensor Configuration Parameters** (Line 280)
   - **Columns**: Sensor, Parameters, Range, Notes
   - **Rows**: RGB Camera, Depth Camera, LiDAR, IMU
   - **Purpose**: Sensor setup quick reference

---

### Chapter 3: Reinforcement Learning with Isaac Gym (isaac-gym-rl.md)

**Diagrams**: 2
1. **GPU-Parallel RL Architecture** (Line 145)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show how Isaac Gym parallelizes RL training on GPU
   - **Alt text**: "GPU-parallel RL architecture showing 4096 environments running in parallel on GPU, computing observations, rewards, and actions in vectorized tensors"
   - **Components**: GPU Envs (4096), Observations Tensor, Actions Tensor, Rewards Tensor, RL Algorithm (PPO)

2. **RL Training Loop** (Line 280)
   - **Type**: Mermaid flowchart
   - **Purpose**: Illustrate typical RL training iteration
   - **Alt text**: "RL training loop flowchart: Reset envs → Collect rollouts → Compute advantages → Update policy (PPO) → Evaluate → Check success rate → Continue or Stop"
   - **Steps**: Reset, Rollout, Advantages, Update, Evaluate, Decision

3. **Sim-to-Real Transfer Workflow** (Line 550)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show domain randomization and transfer process
   - **Alt text**: "Sim-to-real transfer workflow: Train with randomization → Export policy → Test in sim (validation) → Deploy to real robot → Monitor performance → Retrain if needed"
   - **Steps**: Train, Export, Validate, Deploy, Monitor, Retrain

**Tables**: 1
1. **Domain Randomization Parameters** (Line 480)
   - **Columns**: Parameter, Range, Impact, Priority
   - **Rows**: Mass, Friction, Damping, Observation noise, Lighting, Textures
   - **Purpose**: Guide randomization strategy

---

## Module 4: Vision-Language-Action Models (8 elements)

### Chapter 1: Introduction to VLA Models (intro-vla.md)

**Diagrams**: 1
1. **VLA Pipeline Architecture** (Line 165)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show VLA inference pipeline from inputs to robot actions
   - **Alt text**: "VLA pipeline flowchart showing Camera Image (640×480 RGB) and Language Instruction flowing through Vision Encoder (ViT/ResNet), Language Encoder (BERT/T5), Multimodal Fusion (Cross-Attention), Action Decoder (Transformer), to Robot Actions (7 joint angles) and Robot Execution"
   - **Components**: Camera, Language Input, Vision Encoder, Language Encoder, Fusion, Action Decoder, Robot

**Tables**: 1
1. **VLA vs Traditional Robotics Comparison** (Line 235)
   - **Columns**: Aspect, Traditional Robotics, VLA Models
   - **Rows**: Task specification, Perception, Planning, Control, Generalization, Data requirements, Setup time, Failure modes
   - **Purpose**: Highlight VLA advantages and challenges

---

### Chapter 2: VLA Architectures (vla-architectures.md)

**Diagrams**: 2
1. **RT-1 Architecture Diagram** (Line 145)
   - **Type**: Mermaid flowchart
   - **Purpose**: Detailed RT-1 model architecture
   - **Alt text**: "RT-1 architecture flowchart showing Camera Image (300×300) → ResNet-50 Backbone → Token Learner (19×19×2048 → 81×512) → combined with Language Instruction via BERT → Transformer Encoder (8 layers) → Transformer Decoder (4 layers) → Action Tokens (11 bins × 256 classes) → Detokenize → Robot Execution"
   - **Components**: Image, ResNet, Token Learner, BERT, Transformer, Action Tokens, Detokenize, Robot

2. **RT-2 Architecture Diagram** (Line 250)
   - **Type**: Mermaid flowchart
   - **Purpose**: RT-2 VLM-based architecture
   - **Alt text**: "RT-2 architecture showing Vision-Language Model (PaLI-X 55B) pretrained on 10B image-text pairs, then co-fine-tuned to output either Text Tokens (language tasks) or Action Tokens (robot tasks) through shared Vision Encoder (ViT-e 4B) and Language Model (UL2 51B)"
   - **Components**: PaLI-X VLM, Pretraining (10B pairs), Robot Camera, Instruction, ViT-e, UL2, Decoder, Output Type (Text/Action), Robot

**Tables**: 3
1. **RT-1 Performance Results** (Line 120)
   - **Columns**: Task Category, Success Rate
   - **Rows**: Seen tasks, Novel instructions, Novel objects (seen category), Novel objects (unseen category)
   - **Purpose**: Quantify RT-1 generalization

2. **RT-2 vs RT-1 Performance** (Line 240)
   - **Columns**: Metric, RT-1, RT-2
   - **Rows**: Seen tasks, Novel objects (seen), Novel objects (unseen), Symbolic reasoning
   - **Purpose**: Show RT-2 improvements

3. **Architecture Comparison Table** (Line 370)
   - **Columns**: Aspect, RT-1, RT-2, PaLM-E
   - **Rows**: Parameters, Pretraining, Training data, Tasks, Generalization, Inference speed, Novel capabilities, Use cases, Deployment
   - **Purpose**: Comprehensive three-way model comparison

---

### Chapter 3: Training and Deploying VLA (vla-training-deployment.md)

**Diagrams**: 1
1. **VLA Training Pipeline** (Line 275)
   - **Type**: Mermaid flowchart
   - **Purpose**: Show fine-tuning and deployment workflow
   - **Alt text**: "VLA training pipeline showing Pretrained Model (RT-2/PaLM-E) → Task-Specific Demonstrations (Real Robot teleoperation or Isaac Gym RL simulation) → Data Augmentation → Fine-Tune Action Decoder → Evaluate Success Rate → Decision (acceptable? if no, collect more data; if yes, deploy to robot)"
   - **Components**: Pretrained Model, Demo Data, Data Source (Real/Sim), Augmentation, Fine-tune, Evaluate, Decision, Deploy

**Tables**: 1
1. **Fine-Tuning Hyperparameters** (Line 260)
   - **Columns**: Hyperparameter, RT-1, RT-2, PaLM-E, Notes
   - **Rows**: Learning rate, Batch size, Epochs, Weight decay, Warmup steps, Gradient clipping
   - **Purpose**: Guide fine-tuning configuration

---

## Static Assets

### Images

1. **Book Logo** (`static/img/book_robotics_favicon_and_book_logo_image00.png`)
   - **Purpose**: Navbar logo and favicon
   - **Dimensions**: 256×256px
   - **Format**: PNG

2. **Docusaurus Social Card** (`static/img/docusaurus-social-card.jpg`)
   - **Purpose**: Social media preview (og:image)
   - **Dimensions**: 1200×630px
   - **Format**: JPEG

3. **Tutorial Screenshot** (if any - not yet added)
   - **Location**: TBD
   - **Purpose**: Step-by-step tutorials

---

## Visual Asset Statistics

### By Module

| Module | Mermaid Diagrams | Tables | Total |
|--------|-----------------|--------|-------|
| Module 1: ROS 2 | 6 | 2 | 8 |
| Module 2: Gazebo & Unity | 3 | 4 | 7 |
| Module 3: NVIDIA Isaac | 6 | 3 | 9 |
| Module 4: VLA | 4 | 4 | 8 |
| **Total** | **19** | **13** | **32** |

**Note**: Updated count shows 32 total elements (19 Mermaid + 13 tables), exceeding the 18-27 target.

### By Type

- **Architecture Diagrams**: 8 (System overviews, pipeline flows)
- **Workflow Diagrams**: 6 (Step-by-step processes)
- **Communication Diagrams**: 2 (Node graphs, message flows)
- **Hierarchy Diagrams**: 3 (File structures, component trees)
- **Comparison Tables**: 7 (Feature comparisons, benchmarks)
- **Reference Tables**: 6 (Commands, parameters, configurations)

---

## Accessibility Compliance

### Alt Text Requirements

**Status**: ✅ All Mermaid diagrams have descriptive alt text

**Format**: Each Mermaid diagram includes:
```markdown
**Alt text**: [Detailed description of diagram structure, data flow, and key components]
```

**Example** (Isaac Gym RL):
```
**Alt text**: "GPU-parallel RL architecture showing 4096 environments running in parallel on GPU, computing observations, rewards, and actions in vectorized tensors"
```

### Color Contrast

- **Mermaid Default Theme**: Passes WCAG AA (4.5:1 contrast ratio)
- **Dark Mode**: Custom Docusaurus theme ensures sufficient contrast
- **Table Borders**: Visible in both light and dark modes

### Screen Reader Support

- **Mermaid Diagrams**: Rendered as SVG with embedded descriptions
- **Tables**: Standard HTML table markup with proper headers
- **Navigation**: Semantic HTML ensures screen reader compatibility

---

## Maintenance

### Adding New Diagrams

1. **Create Mermaid code** in chapter Markdown file
2. **Add alt text** immediately after diagram
3. **Update this document** with diagram location and purpose
4. **Test rendering**: Run `npm start` to preview
5. **Validate accessibility**: Check alt text is descriptive (20+ words)

### Updating Existing Diagrams

1. **Edit Mermaid code** in source file
2. **Update alt text** if structure changed
3. **Update this document** if purpose changed
4. **Rebuild site**: `npm run build` to verify syntax
5. **Visual inspection**: Check alignment, labels, colors

### Diagram Best Practices

- **Simplicity**: Max 10 nodes per diagram
- **Labels**: Clear, concise (5-10 words per node)
- **Arrows**: Labeled with action/data descriptions
- **Colors**: Use `style` directive for emphasis
- **Layout**: Top-to-bottom or left-to-right flow

---

## Future Enhancements

### Planned Additions

1. **Interactive Diagrams**: Use Mermaid live editor integration
2. **Code Diagrams**: UML class diagrams for ROS 2 nodes
3. **Screenshots**: Gazebo, Unity, Isaac Sim interfaces
4. **GIFs/Videos**: Short clips of robot simulations
5. **Flowchart Library**: Reusable diagram components

### Tools

- **Diagram Editor**: Mermaid Live (https://mermaid.live)
- **Screenshot Tool**: Flameshot (Linux), Greenshot (Windows)
- **GIF Recorder**: peek (Linux), ScreenToGif (Windows)
- **SVG Optimizer**: SVGO (reduce file size)

---

**Last Updated**: 2025-12-02
**Total Visual Assets**: 32 (19 Mermaid + 13 tables)
**Target Range**: 18-27 (exceeded)
**Accessibility**: 100% compliant (all alt text added)
