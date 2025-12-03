# Tasks: Book Completion - Modules 2-4, Search & Deployment

**Input**: Design documents from `specs/003-book-completion/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: Not included - content validation will be manual + build validation

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

**Documentation project**: `docs/`, `static/img/`, `.github/workflows/` at repository root
**Spec directory**: `specs/003-book-completion/` for planning artifacts

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure for new modules

- [X] T001 Create `docs/module-2-gazebo-unity/` directory for Module 2 chapters
- [X] T002 Create `docs/module-3-isaac/` directory for Module 3 chapters
- [X] T003 Create `docs/module-4-vla/` directory for Module 4 chapters
- [X] T004 [P] Create `static/img/module-2/` directory for Module 2 images
- [X] T005 [P] Create `static/img/module-3/` directory for Module 3 images
- [X] T006 [P] Create `static/img/module-4/` directory for Module 4 images

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Research and design artifacts that inform all content creation

**⚠️ CRITICAL**: No module writing can begin until research and content outlines are complete

- [X] T007 Research Gazebo simulation best practices (Gazebo Garden vs. Fortress, ROS 2 Humble integration) and document in `specs/003-book-completion/research.md`
- [X] T008 Research Unity Robotics Hub setup (Unity 2022 LTS, URDF Importer, ROS TCP Connector) and document in `specs/003-book-completion/research.md`
- [X] T009 Research NVIDIA Isaac installation & setup (Isaac Sim/Gym, GPU requirements, ROS 2 integration) and document in `specs/003-book-completion/research.md`
- [X] T010 Research VLA model state-of-the-art (RT-1, RT-2, PaLM-E architectures, deployment) and document in `specs/003-book-completion/research.md`
- [X] T011 Research search plugin selection (local vs. Algolia) and document decision in `specs/003-book-completion/research.md`
- [X] T012 Research GitHub Pages deployment workflow (GitHub Actions template, error handling) and document in `specs/003-book-completion/research.md`
- [X] T013 Create detailed content outlines for all 9 chapters in `specs/003-book-completion/content-outline.md` (section breakdown, word counts, visual assets, code examples per chapter)

**Checkpoint**: Foundation ready - module content creation can now begin in parallel

---

## Phase 3: User Story 1 - Complete Module 2: Gazebo & Unity Content (Priority: P1) 🎯 MVP

**Goal**: Write 3 complete chapters on robot simulation with Gazebo and Unity, including installation guides, architecture diagrams, code examples, and hands-on exercises.

**Independent Test**: Verify all 3 Module 2 chapters are accessible at `/docs/module-2-gazebo-unity/*`, contain 1500-2500 words each, include 2-3 diagrams per chapter, and have working code examples. Students can complete exercises independently.

### Implementation for User Story 1

**Chapter 1: Introduction to Gazebo**

- [X] T014 [P] [US1] Write frontmatter and prerequisites section for `docs/module-2-gazebo-unity/intro-gazebo.md` (sidebar_position: 1, prerequisites: Module 1 complete)
- [X] T015 [P] [US1] Write introduction section for `docs/module-2-gazebo-unity/intro-gazebo.md` (150 words: why Gazebo matters, chapter goals, learning objectives)
- [X] T016 [US1] Write "What is Gazebo" section in `docs/module-2-gazebo-unity/intro-gazebo.md` (300-400 words: Gazebo overview, use cases, advantages for ROS 2 testing)
- [X] T017 [US1] Write "Gazebo Architecture" section in `docs/module-2-gazebo-unity/intro-gazebo.md` (400-500 words: physics engine, scene graph, rendering, sensors)
- [X] T018 [US1] Create Gazebo architecture Mermaid diagram in `docs/module-2-gazebo-unity/intro-gazebo.md` (graph TD: Physics → Scene Graph → Rendering → Sensors, with alt text)
- [X] T019 [US1] Write "Installing Gazebo Garden" section in `docs/module-2-gazebo-unity/intro-gazebo.md` (400-500 words: Ubuntu 22.04 installation, ROS 2 Humble integration, ros_gz_bridge setup)
- [X] T020 [US1] Write "First Gazebo Simulation" section in `docs/module-2-gazebo-unity/intro-gazebo.md` (300-400 words: launching Gazebo, loading robot model, basic controls)
- [X] T021 [US1] Add code example for Gazebo launch file in `docs/module-2-gazebo-unity/intro-gazebo.md` (Python launch API, spawn robot model, configure world)
- [X] T022 [US1] Create Gazebo workflow Mermaid diagram in `docs/module-2-gazebo-unity/intro-gazebo.md` (sequence diagram: Launch → Load Model → Start Physics → Publish Topics)
- [X] T023 [US1] Write hands-on exercises section for `docs/module-2-gazebo-unity/intro-gazebo.md` (4 exercises: install Gazebo, launch empty world, spawn robot, control via ROS 2 topics)
- [X] T024 [US1] Write key takeaways section for `docs/module-2-gazebo-unity/intro-gazebo.md` (7 bullet points summarizing Gazebo concepts)
- [X] T025 [US1] Add navigation links to `docs/module-2-gazebo-unity/intro-gazebo.md` (Previous: Module 1 Chapter 3, Next: Module 2 Chapter 2)

**Chapter 2: Unity for Robotics**

- [X] T026 [P] [US1] Write frontmatter and prerequisites section for `docs/module-2-gazebo-unity/unity-robotics.md` (sidebar_position: 2, prerequisites: Gazebo basics, Unity basics)
- [X] T027 [P] [US1] Write introduction section for `docs/module-2-gazebo-unity/unity-robotics.md` (150 words: why Unity for robotics, chapter goals, learning objectives)
- [X] T028 [US1] Write "Unity Robotics Hub Overview" section in `docs/module-2-gazebo-unity/unity-robotics.md` (300-400 words: Unity Hub purpose, ROS 2 integration, advantages)
- [X] T029 [US1] Write "Installing Unity 2022 LTS" section in `docs/module-2-gazebo-unity/unity-robotics.md` (400-500 words: Unity Hub install, Unity 2022 LTS setup, Robotics Hub package)
- [X] T030 [US1] Write "URDF Importer for Unity" section in `docs/module-2-gazebo-unity/unity-robotics.md` (400-500 words: importing robot models, articulation body setup, material configuration)
- [X] T031 [US1] Create Unity Robotics Hub Mermaid diagram in `docs/module-2-gazebo-unity/unity-robotics.md` (sequence diagram: Unity ↔ ROS TCP Connector ↔ ROS 2, with alt text)
- [X] T032 [US1] Write "ROS 2 Communication" section in `docs/module-2-gazebo-unity/unity-robotics.md` (300-400 words: ROS TCP Connector, publisher/subscriber in Unity, service calls)
- [X] T033 [US1] Add code example for Unity ROS 2 publisher in `docs/module-2-gazebo-unity/unity-robotics.md` (C# script publishing sensor data to ROS 2 topic)
- [X] T034 [US1] Write hands-on exercises section for `docs/module-2-gazebo-unity/unity-robotics.md` (4 exercises: install Unity Hub, import URDF, publish topic, subscribe from ROS 2)
- [X] T035 [US1] Write key takeaways section for `docs/module-2-gazebo-unity/unity-robotics.md` (7 bullet points summarizing Unity Robotics Hub)
- [X] T036 [US1] Add navigation links to `docs/module-2-gazebo-unity/unity-robotics.md` (Previous: Module 2 Chapter 1, Next: Module 2 Chapter 3)

**Chapter 3: Simulation Best Practices**

- [X] T037 [P] [US1] Write frontmatter and prerequisites section for `docs/module-2-gazebo-unity/simulation-best-practices.md` (sidebar_position: 3, prerequisites: Gazebo and Unity basics)
- [X] T038 [P] [US1] Write introduction section for `docs/module-2-gazebo-unity/simulation-best-practices.md` (150 words: importance of simulation testing, chapter goals, learning objectives)
- [X] T039 [US1] Write "Simulation Testing Workflow" section in `docs/module-2-gazebo-unity/simulation-best-practices.md` (400-500 words: unit tests, integration tests, sim-to-real gap)
- [X] T040 [US1] Create simulation testing Mermaid diagram in `docs/module-2-gazebo-unity/simulation-best-practices.md` (flowchart: Unit Tests → Integration Tests → Sim Tests → Real Robot)
- [X] T041 [US1] Write "Performance Optimization" section in `docs/module-2-gazebo-unity/simulation-best-practices.md` (300-400 words: physics timestep, rendering optimization, parallel simulations)
- [X] T042 [US1] Write "Troubleshooting Common Issues" section in `docs/module-2-gazebo-unity/simulation-best-practices.md` (300-400 words: physics instability, sensor noise, model collisions)
- [X] T043 [US1] Add code example for simulation testing script in `docs/module-2-gazebo-unity/simulation-best-practices.md` (Python script testing robot navigation in simulation)
- [X] T044 [US1] Write hands-on exercises section for `docs/module-2-gazebo-unity/simulation-best-practices.md` (5 exercises: write sim test, optimize physics, debug collision, measure performance, validate sensor data)
- [X] T045 [US1] Write key takeaways section for `docs/module-2-gazebo-unity/simulation-best-practices.md` (7 bullet points summarizing simulation best practices)
- [X] T046 [US1] Add navigation links to `docs/module-2-gazebo-unity/simulation-best-practices.md` (Previous: Module 2 Chapter 2, Next: Module 3 Chapter 1)

**Validation for User Story 1**

- [X] T047 [US1] Validate Module 2 Chapter 1 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-2-gazebo-unity/intro-gazebo.md`
- [X] T048 [US1] Validate Module 2 Chapter 2 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-2-gazebo-unity/unity-robotics.md`
- [X] T049 [US1] Validate Module 2 Chapter 3 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-2-gazebo-unity/simulation-best-practices.md`
- [X] T050 [US1] Verify Module 2 has 6-9 visual elements (Mermaid diagrams + figure captions) across all 3 chapters
- [X] T051 [US1] Verify all Gazebo/Unity code examples are tested and error-free (install Gazebo/Unity, run examples, confirm output)
- [X] T052 [US1] Run `npm run build` to validate Module 2 chapters build without errors (no broken links, missing images, invalid Mermaid)

**Checkpoint**: At this point, Module 2 (User Story 1) should be fully functional and testable independently

---

## Phase 4: User Story 2 - Complete Module 3: NVIDIA Isaac Content (Priority: P2)

**Goal**: Write 3 complete chapters on NVIDIA Isaac for AI-driven robotics, including Isaac Sim/Gym setup, RL training examples, and deployment workflows.

**Independent Test**: Verify all 3 Module 3 chapters are accessible at `/docs/module-3-isaac/*`, contain 1500-2500 words each, include 2-3 diagrams per chapter, and demonstrate Isaac Sim/Gym workflows. Students can set up Isaac environments and run basic RL training.

### Implementation for User Story 2

**Chapter 1: Introduction to NVIDIA Isaac**

- [X] T053 [P] [US2] Write frontmatter and prerequisites section for `docs/module-3-isaac/intro-isaac.md` (sidebar_position: 1, prerequisites: Module 2 complete, GPU available)
- [X] T054 [P] [US2] Write introduction section for `docs/module-3-isaac/intro-isaac.md` (150 words: why Isaac for AI robotics, chapter goals, learning objectives)
- [X] T055 [US2] Write "What is NVIDIA Isaac" section in `docs/module-3-isaac/intro-isaac.md` (300-400 words: Isaac Sim vs. Isaac Gym, use cases, GPU-accelerated simulation)
- [X] T056 [US2] Write "Isaac Ecosystem Overview" section in `docs/module-3-isaac/intro-isaac.md` (400-500 words: Omniverse, USD format, Isaac SDK, Isaac ROS)
- [X] T057 [US2] Create Isaac ecosystem Mermaid diagram in `docs/module-3-isaac/intro-isaac.md` (graph: Omniverse → USD → Isaac Sim → ROS 2, with alt text)
- [X] T058 [US2] Write "System Requirements" section in `docs/module-3-isaac/intro-isaac.md` (300-400 words: GPU requirements, CUDA version, VRAM, Ubuntu 22.04)
- [X] T059 [US2] Write "Installing Isaac Sim" section in `docs/module-3-isaac/intro-isaac.md` (400-500 words: Omniverse Launcher, Isaac Sim download, first launch)
- [X] T060 [US2] Add code example for Isaac Sim setup verification in `docs/module-3-isaac/intro-isaac.md` (Python script checking Isaac Sim installation, loading sample scene)
- [X] T061 [US2] Write hands-on exercises section for `docs/module-3-isaac/intro-isaac.md` (4 exercises: verify GPU, install Isaac Sim, load sample scene, explore UI)
- [X] T062 [US2] Write key takeaways section for `docs/module-3-isaac/intro-isaac.md` (7 bullet points summarizing Isaac ecosystem)
- [X] T063 [US2] Add navigation links to `docs/module-3-isaac/intro-isaac.md` (Previous: Module 2 Chapter 3, Next: Module 3 Chapter 2)

**Chapter 2: Isaac Sim for Robotics**

- [X] T064 [P] [US2] Write frontmatter and prerequisites section for `docs/module-3-isaac/isaac-sim.md` (sidebar_position: 2, prerequisites: Isaac Sim installed, USD basics)
- [X] T065 [P] [US2] Write introduction section for `docs/module-3-isaac/isaac-sim.md` (150 words: Isaac Sim capabilities, chapter goals, learning objectives)
- [X] T066 [US2] Write "Creating Simulation Scenes" section in `docs/module-3-isaac/isaac-sim.md` (400-500 words: USD stage, prims, scene composition, asset import)
- [X] T067 [US2] Write "Importing Robot Models" section in `docs/module-3-isaac/isaac-sim.md` (400-500 words: URDF import, articulation configuration, physics properties)
- [X] T068 [US2] Create Isaac Sim workflow Mermaid diagram in `docs/module-3-isaac/isaac-sim.md` (flowchart: Create Scene → Import Robot → Configure Physics → Add Sensors → Run Simulation)
- [X] T069 [US2] Write "Sensor Simulation" section in `docs/module-3-isaac/isaac-sim.md` (300-400 words: cameras, LiDAR, IMU, force sensors, synthetic data generation)
- [X] T070 [US2] Write "ROS 2 Integration" section in `docs/module-3-isaac/isaac-sim.md` (400-500 words: Isaac ROS packages, ROS 2 bridge, topic publishing, service calls)
- [X] T071 [US2] Add code example for Isaac Sim ROS 2 integration in `docs/module-3-isaac/isaac-sim.md` (Python script controlling robot in Isaac Sim via ROS 2 topics)
- [X] T072 [US2] Write hands-on exercises section for `docs/module-3-isaac/isaac-sim.md` (5 exercises: create scene, import robot, add camera sensor, publish ROS 2 topics, collect synthetic data)
- [X] T073 [US2] Write key takeaways section for `docs/module-3-isaac/isaac-sim.md` (7 bullet points summarizing Isaac Sim usage)
- [X] T074 [US2] Add navigation links to `docs/module-3-isaac/isaac-sim.md` (Previous: Module 3 Chapter 1, Next: Module 3 Chapter 3)

**Chapter 3: Reinforcement Learning with Isaac Gym**

- [X] T075 [P] [US2] Write frontmatter and prerequisites section for `docs/module-3-isaac/isaac-gym-rl.md` (sidebar_position: 3, prerequisites: Isaac Sim basics, RL basics, PyTorch)
- [X] T076 [P] [US2] Write introduction section for `docs/module-3-isaac/isaac-gym-rl.md` (150 words: why RL for robotics, chapter goals, learning objectives)
- [X] T077 [US2] Write "Isaac Gym Overview" section in `docs/module-3-isaac/isaac-gym-rl.md` (300-400 words: parallel environments, GPU-accelerated training, RL algorithms)
- [X] T078 [US2] Write "RL Training Workflow" section in `docs/module-3-isaac/isaac-gym-rl.md` (400-500 words: environment setup, reward function, PPO/SAC algorithms, training loop)
- [X] T079 [US2] Create Isaac Gym RL Mermaid diagram in `docs/module-3-isaac/isaac-gym-rl.md` (sequence diagram: Environment → Observation → Policy → Action → Reward → Update Policy)
- [X] T080 [US2] Write "Training Example: Reaching Task" section in `docs/module-3-isaac/isaac-gym-rl.md` (400-500 words: task definition, observation space, action space, reward shaping)
- [X] T081 [US2] Add code example for Isaac Gym RL training in `docs/module-3-isaac/isaac-gym-rl.md` (Python script training robot arm to reach target using PPO)
- [X] T082 [US2] Create sim-to-real transfer Mermaid diagram in `docs/module-3-isaac/isaac-gym-rl.md` (flowchart: Simulation Training → Domain Randomization → Policy Transfer → Real Robot Deployment)
- [X] T083 [US2] Write hands-on exercises section for `docs/module-3-isaac/isaac-gym-rl.md` (5 exercises: setup Isaac Gym, define RL task, train policy, evaluate performance, visualize results)
- [X] T084 [US2] Write key takeaways section for `docs/module-3-isaac/isaac-gym-rl.md` (7 bullet points summarizing Isaac Gym RL)
- [X] T085 [US2] Add navigation links to `docs/module-3-isaac/isaac-gym-rl.md` (Previous: Module 3 Chapter 2, Next: Module 4 Chapter 1)

**Validation for User Story 2**

- [X] T086 [US2] Validate Module 3 Chapter 1 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-3-isaac/intro-isaac.md`
- [X] T087 [US2] Validate Module 3 Chapter 2 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-3-isaac/isaac-sim.md`
- [X] T088 [US2] Validate Module 3 Chapter 3 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-3-isaac/isaac-gym-rl.md`
- [X] T089 [US2] Verify Module 3 has 6-9 visual elements (Mermaid diagrams + figure captions) across all 3 chapters
- [X] T090 [US2] Verify all Isaac Sim/Gym code examples are documented with expected behavior (note: actual testing requires NVIDIA GPU)
- [X] T091 [US2] Run `npm run build` to validate Module 3 chapters build without errors (no broken links, missing images, invalid Mermaid)

**Checkpoint**: At this point, Module 3 (User Story 2) should be fully functional and testable independently

---

## Phase 5: User Story 3 - Complete Module 4: Vision-Language-Action (VLA) Content (Priority: P3)

**Goal**: Write 3 complete chapters on VLA models for cutting-edge physical AI, including RT-1, RT-2, PaLM-E architectures, training workflows, and deployment considerations.

**Independent Test**: Verify all 3 Module 4 chapters are accessible at `/docs/module-4-vla/*`, contain 1500-2500 words each, include 2-3 diagrams per chapter, and explain VLA architecture, training, and deployment. Students understand state-of-the-art VLA models like RT-1, RT-2, and PaLM-E.

### Implementation for User Story 3

**Chapter 1: Introduction to VLA Models**

- [x] T092 [P] [US3] Write frontmatter and prerequisites section for `docs/module-4-vla/intro-vla.md` (sidebar_position: 1, prerequisites: Module 3 complete, transformer basics, multimodal AI)
- [x] T093 [P] [US3] Write introduction section for `docs/module-4-vla/intro-vla.md` (150 words: why VLA for robotics, chapter goals, learning objectives)
- [x] T094 [US3] Write "What are VLA Models" section in `docs/module-4-vla/intro-vla.md` (400-500 words: vision-language-action definition, multimodal learning, embodied AI)
- [x] T095 [US3] Write "VLA Use Cases" section in `docs/module-4-vla/intro-vla.md` (300-400 words: manipulation tasks, natural language control, household robots, industrial automation)
- [x] T096 [US3] Create VLA pipeline Mermaid diagram in `docs/module-4-vla/intro-vla.md` (flowchart: Image/Language Input → Vision Encoder → Language Model → Action Decoder → Robot Actions, with alt text)
- [x] T097 [US3] Write "Evolution of VLA Models" section in `docs/module-4-vla/intro-vla.md` (400-500 words: early approaches, transformer revolution, RT-1/RT-2/PaLM-E timeline)
- [x] T098 [US3] Write "VLA vs Traditional Robotics" section in `docs/module-4-vla/intro-vla.md` (300-400 words: comparison table, advantages, challenges, future directions)
- [x] T099 [US3] Write hands-on exercises section for `docs/module-4-vla/intro-vla.md` (3 exercises: analyze VLA paper, compare VLA architectures, identify use cases)
- [x] T100 [US3] Write key takeaways section for `docs/module-4-vla/intro-vla.md` (7 bullet points summarizing VLA concepts)
- [x] T101 [US3] Add navigation links to `docs/module-4-vla/intro-vla.md` (Previous: Module 3 Chapter 3, Next: Module 4 Chapter 2)

**Chapter 2: VLA Architectures (RT-1, RT-2, PaLM-E)**

- [x] T102 [P] [US3] Write frontmatter and prerequisites section for `docs/module-4-vla/vla-architectures.md` (sidebar_position: 2, prerequisites: VLA basics, transformer architecture)
- [x] T103 [P] [US3] Write introduction section for `docs/module-4-vla/vla-architectures.md` (150 words: chapter overview, architecture comparison goals, learning objectives)
- [x] T104 [US3] Write "RT-1: Robotics Transformer 1" section in `docs/module-4-vla/vla-architectures.md` (500-600 words: RT-1 architecture, token learner, transformer encoder-decoder, action tokens, training data)
- [x] T105 [US3] Create RT-1 architecture Mermaid diagram in `docs/module-4-vla/vla-architectures.md` (graph: Image/Text → Token Learner → Transformer → Action Tokens → Robot Control, with alt text)
- [x] T106 [US3] Write "RT-2: Robotics Transformer 2" section in `docs/module-4-vla/vla-architectures.md` (500-600 words: RT-2 improvements, vision-language model base, web-scale pretraining, emergent capabilities)
- [x] T107 [US3] Create RT-2 architecture Mermaid diagram in `docs/module-4-vla/vla-architectures.md` (graph: Vision-Language Model (PaLI-X) → Fine-tuning → Action Decoder → Robot Actions, with alt text)
- [x] T108 [US3] Write "PaLM-E: Embodied Language Model" section in `docs/module-4-vla/vla-architectures.md` (400-500 words: PaLM-E architecture, multimodal integration, state estimation, planning capabilities)
- [x] T109 [US3] Create architecture comparison table in `docs/module-4-vla/vla-architectures.md` (markdown table: RT-1 vs RT-2 vs PaLM-E - parameters, training data, capabilities, limitations)
- [x] T110 [US3] Write hands-on exercises section for `docs/module-4-vla/vla-architectures.md` (4 exercises: read RT-1 paper, compare RT-2 improvements, analyze PaLM-E capabilities, reproduce architecture diagram)
- [x] T111 [US3] Write key takeaways section for `docs/module-4-vla/vla-architectures.md` (7 bullet points summarizing VLA architectures)
- [x] T112 [US3] Add navigation links to `docs/module-4-vla/vla-architectures.md` (Previous: Module 4 Chapter 1, Next: Module 4 Chapter 3)

**Chapter 3: Training and Deploying VLA Systems**

- [x] T113 [P] [US3] Write frontmatter and prerequisites section for `docs/module-4-vla/vla-training-deployment.md` (sidebar_position: 3, prerequisites: VLA architectures, deep learning frameworks)
- [x] T114 [P] [US3] Write introduction section for `docs/module-4-vla/vla-training-deployment.md` (150 words: training challenges, deployment considerations, chapter goals)
- [x] T115 [US3] Write "VLA Training Data Requirements" section in `docs/module-4-vla/vla-training-deployment.md` (400-500 words: demonstration data, teleoperation, data augmentation, synthetic data from Isaac Gym)
- [x] T116 [US3] Write "Fine-Tuning VLA Models" section in `docs/module-4-vla/vla-training-deployment.md` (400-500 words: transfer learning, domain adaptation, task-specific fine-tuning, hyperparameters)
- [x] T117 [US3] Create VLA training pipeline Mermaid diagram in `docs/module-4-vla/vla-training-deployment.md` (flowchart: Pretrained Model → Collect Demo Data → Fine-tune → Evaluate → Deploy, with alt text)
- [x] T118 [US3] Write "Deployment Considerations" section in `docs/module-4-vla/vla-training-deployment.md` (400-500 words: model size, inference time, edge devices, safety constraints, failure modes)
- [x] T119 [US3] Write "Real-World Case Studies" section in `docs/module-4-vla/vla-training-deployment.md` (300-400 words: RT-1 in Google's Everyday Robots, RT-2 emergent reasoning, industrial applications)
- [x] T120 [US3] Add code example for VLA inference workflow in `docs/module-4-vla/vla-training-deployment.md` (Python pseudocode showing image/text → VLA model → action output)
- [x] T121 [US3] Write hands-on exercises section for `docs/module-4-vla/vla-training-deployment.md` (5 exercises: analyze demo dataset, design fine-tuning strategy, estimate inference time, identify safety constraints, propose deployment architecture)
- [x] T122 [US3] Write key takeaways section for `docs/module-4-vla/vla-training-deployment.md` (7 bullet points summarizing VLA training and deployment)
- [x] T123 [US3] Add navigation links to `docs/module-4-vla/vla-training-deployment.md` (Previous: Module 4 Chapter 2, Next: [End of book or future modules])

**Validation for User Story 3**

- [x] T124 [US3] Validate Module 4 Chapter 1 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-4-vla/intro-vla.md`
- [x] T125 [US3] Validate Module 4 Chapter 2 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-4-vla/vla-architectures.md`
- [x] T126 [US3] Validate Module 4 Chapter 3 word count is 1500-2500 words (prose excluding code) using `wc -w docs/module-4-vla/vla-training-deployment.md`
- [x] T127 [US3] Verify Module 4 has 6-9 visual elements (Mermaid diagrams + figure captions + comparison table) across all 3 chapters
- [x] T128 [US3] Verify all VLA architectures are accurately described based on official papers (RT-1, RT-2, PaLM-E)
- [x] T129 [US3] Run `npm run build` to validate Module 4 chapters build without errors (no broken links, missing images, invalid Mermaid)

**Checkpoint**: At this point, Module 4 (User Story 3) should be fully functional and testable independently. All content chapters (Modules 1-4) are now complete.

---

## Phase 6: User Story 4 - Add Search Functionality (Priority: P4)

**Goal**: Integrate search functionality (local plugin or Algolia) to enable students to quickly find topics, commands, and concepts across all 12 chapters.

**Independent Test**: Enter search query (e.g., "ROS 2 topics", "Gazebo launch file", "Isaac Gym") and verify that relevant results appear with highlighted keywords and links to correct pages within 1 second.

### Implementation for User Story 4

- [x] T130 [US4] Install @easyops-cn/docusaurus-search-local plugin via `npm install --save @easyops-cn/docusaurus-search-local`
- [x] T131 [US4] Configure search plugin in `docusaurus.config.js` (add theme with hashed: true, language: ["en"], indexDocs: true, highlightSearchTermsOnTargetPage: true)
- [x] T132 [US4] Create search configuration documentation in `specs/003-book-completion/search-config.md` (document plugin choice rationale, configuration options, testing steps)
- [x] T133 [US4] Run `npm run build` to generate search index for all 12 chapters (Module 1-4)
- [x] T134 [US4] Test search functionality locally with `npm start` (search for "ROS 2", "Gazebo", "Isaac Sim", "VLA", verify results appear with keyword highlighting)
- [x] T135 [US4] Verify search bar appears in navigation header on all pages
- [x] T136 [US4] Verify search results link to correct sections and highlight keywords
- [x] T137 [US4] Verify search results appear within 1 second for queries under 50 characters
- [x] T138 [US4] Test edge cases: no results message, special characters in queries, exact match with quotes (e.g., "ros2 topic pub")

**Checkpoint**: At this point, search functionality (User Story 4) should be fully functional and testable independently

---

## Phase 7: User Story 5 - Deploy to GitHub Pages (Priority: P5)

**Goal**: Configure automated GitHub Pages deployment via GitHub Actions workflow so the textbook is accessible at a public URL from any device.

**Independent Test**: Visit deployed GitHub Pages URL (e.g., https://username.github.io/physical-ai-humanoid-robotics/) and verify all 12 chapters load correctly, navigation works, search functions, and images render properly.

### Implementation for User Story 5

- [x] T139 [US5] Create GitHub Actions deployment workflow file at `.github/workflows/deploy.yml` (workflow triggered on push to main branch)
- [x] T140 [US5] Configure deploy.yml build job (checkout code, setup Node.js 18, npm ci, npm run build, upload artifact)
- [x] T141 [US5] Configure deploy.yml deploy job (needs: build, deploy to GitHub Pages using actions/deploy-pages@v4)
- [x] T142 [US5] Add build validation to deploy.yml (build must fail on missing images, broken Mermaid, broken internal links)
- [x] T143 [US5] Add error handling to deploy.yml (if build fails, deployment doesn't run, GitHub sends notification)
- [x] T144 [US5] Update `docusaurus.config.js` with correct GitHub Pages URL and baseUrl (e.g., url: 'https://username.github.io', baseUrl: '/repo-name/')
- [x] T145 [US5] Create deployment configuration documentation in `specs/003-book-completion/deployment-config.md` (document workflow structure, error handling, rollback strategy)
- [x] T146 [US5] Enable GitHub Pages in repository settings (Settings → Pages → Source: GitHub Actions)
- [x] T147 [US5] Test deployment workflow by pushing to main branch (verify GitHub Actions runs, build succeeds, site deploys within 10 minutes)
- [x] T148 [US5] Verify deployed site loads at GitHub Pages URL with all 12 chapters accessible
- [x] T149 [US5] Verify all internal links work correctly on deployed site (Previous/Next navigation, module links, anchor links)
- [x] T150 [US5] Verify all images load without errors on deployed site (check all modules for broken image links)
- [x] T151 [US5] Verify search functionality works on deployed site (test same queries as T134)
- [x] T152 [US5] Test responsive design on deployed site (mobile 320px, tablet 768px, desktop 1920px)

**Checkpoint**: At this point, deployment (User Story 5) should be fully functional. The complete textbook is now accessible via GitHub Pages.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, optimization, and documentation updates affecting all modules

- [x] T153 [P] Validate total word count for all 12 chapters is 18,000-30,000 words (Module 1: ~12,700 + Modules 2-4: 5,300-17,300)
- [x] T154 [P] Validate total visual elements count is 18-27 diagrams (Module 1: 8 + Modules 2-4: 10-19)
- [x] T155 [P] Validate total exercises count is 27-45 exercises (Module 1: 13 + Modules 2-4: 14-32)
- [x] T156 [P] Create visual assets plan documentation in `specs/003-book-completion/visual-assets-plan.md` (document all 18-27 diagrams with locations, captions, alt text similar to Module 1's visual-assets-plan.md)
- [x] T157 Run Lighthouse audit on deployed site (verify Performance score >90, Accessibility score >90)
- [x] T158 Verify page load time <3 seconds on deployed site (test homepage, module pages, chapter pages)
- [x] T159 Verify all external documentation links are current and valid (ROS 2 docs, Gazebo docs, Unity docs, Isaac docs)
- [x] T160 [P] Update main README.md with link to deployed GitHub Pages site and book description
- [x] T161 [P] Update specs/003-book-completion/content-outline.md with final word counts for Modules 2-4 (similar to how Module 1's content-outline.md was updated in T096)
- [x] T162 Create final validation summary documenting all success criteria met (SC-001 to SC-016 from spec.md)
- [x] T163 Test dark mode toggle on all 12 chapters (verify contrast, readability, image visibility)
- [x] T164 Verify keyboard navigation works on all pages (Tab, Enter, Arrow keys for navigation)
- [x] T165 Run final build validation: `npm run build` completes in <2 minutes without errors

**Checkpoint**: Book is complete, validated, and production-ready

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories (research must inform content creation)
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User Story 1 (Module 2): Can start after Foundational - No dependencies on other stories
  - User Story 2 (Module 3): Can start after Foundational - No dependencies on other stories (but recommended after US1 for progressive learning)
  - User Story 3 (Module 4): Can start after Foundational - No dependencies on other stories (but recommended after US2 for progressive learning)
  - User Story 4 (Search): Can start after Foundational - Recommended after US1-US3 complete (needs content to index)
  - User Story 5 (Deployment): Can start after Foundational - Recommended after US1-US4 complete (needs content to deploy)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Module 2)**: Can start after Foundational (T007-T013) - Independently testable
- **User Story 2 (P2 - Module 3)**: Can start after Foundational - Recommended after US1 for logical progression
- **User Story 3 (P3 - Module 4)**: Can start after Foundational - Recommended after US2 for logical progression
- **User Story 4 (P4 - Search)**: Can start after Foundational - Recommended after US1-US3 to have content to index
- **User Story 5 (P5 - Deployment)**: Can start after Foundational - Recommended after US1-US4 to have complete site to deploy

### Within Each User Story

**Module Content (US1, US2, US3)**:
- Frontmatter → Introduction → Main sections → Diagrams → Exercises → Takeaways → Navigation
- Chapters within a module can be written in parallel (different files)
- Validation tasks run after all chapters complete

**Search (US4)**:
- Install → Configure → Build → Test → Validate

**Deployment (US5)**:
- Create workflow → Configure → Test → Validate live site

### Parallel Opportunities

- **Phase 1 (Setup)**: All 6 directory creation tasks (T001-T006) can run in parallel [P]
- **Phase 2 (Foundational)**: Research tasks (T007-T012) can run in parallel if distributed to different researchers
- **Phase 3 (Module 2)**: Chapter frontmatter/intro tasks (T014, T015, T026, T027, T037, T038) can run in parallel [P]
- **Phase 3 (Module 2)**: All 3 chapters can be written in parallel by different authors (T014-T025, T026-T036, T037-T046 can overlap)
- **Phase 4 (Module 3)**: Chapter frontmatter/intro tasks (T053, T054, T064, T065, T075, T076) can run in parallel [P]
- **Phase 4 (Module 3)**: All 3 chapters can be written in parallel by different authors (T053-T063, T064-T074, T075-T085 can overlap)
- **Phase 5 (Module 4)**: Chapter frontmatter/intro tasks (T092, T093, T102, T103, T113, T114) can run in parallel [P]
- **Phase 5 (Module 4)**: All 3 chapters can be written in parallel by different authors (T092-T101, T102-T112, T113-T123 can overlap)
- **Phase 8 (Polish)**: Validation tasks (T153, T154, T155, T156, T160, T161) can run in parallel [P]

**Full Parallelization** (if 3+ authors available):
- After Foundational phase completes, User Stories 1, 2, 3 can proceed in parallel
- After US1-US3 complete, User Stories 4 and 5 can proceed in parallel

---

## Parallel Example: User Story 1 (Module 2)

```bash
# Launch all Chapter 1 intro tasks together:
Task: "Write frontmatter and prerequisites section for docs/module-2-gazebo-unity/intro-gazebo.md"
Task: "Write introduction section for docs/module-2-gazebo-unity/intro-gazebo.md"

# Launch all Chapter 2 intro tasks together:
Task: "Write frontmatter and prerequisites section for docs/module-2-gazebo-unity/unity-robotics.md"
Task: "Write introduction section for docs/module-2-gazebo-unity/unity-robotics.md"

# Launch all Chapter 3 intro tasks together:
Task: "Write frontmatter and prerequisites section for docs/module-2-gazebo-unity/simulation-best-practices.md"
Task: "Write introduction section for docs/module-2-gazebo-unity/simulation-best-practices.md"

# If 3 authors available, entire Module 2 can be written in parallel:
Author A: Chapter 1 (T014-T025)
Author B: Chapter 2 (T026-T036)
Author C: Chapter 3 (T037-T046)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Module 2)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T013) - CRITICAL research phase
3. Complete Phase 3: User Story 1 - Module 2 (T014-T052)
4. **STOP and VALIDATE**: Test Module 2 independently (all 3 chapters accessible, word counts met, diagrams present, exercises complete)
5. Deploy/demo Module 2 if ready

**MVP Deliverable**: Complete textbook with Modules 1-2 (6 chapters total)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Module 2 (US1) → Test independently → **Book has 6 chapters (MVP!)**
3. Add Module 3 (US2) → Test independently → **Book has 9 chapters**
4. Add Module 4 (US3) → Test independently → **Book has 12 chapters (complete content)**
5. Add Search (US4) → Test independently → **Book has search functionality**
6. Add Deployment (US5) → Test independently → **Book is publicly accessible via GitHub Pages**
7. Each user story adds value without breaking previous stories

### Parallel Team Strategy

With 3 content authors + 1 integrator:

1. **Phase 1-2**: Team completes Setup + Foundational together (1-2 hours)
2. **Phase 3-5** (parallel content creation, 12-15 hours):
   - Author A: Module 2 - Gazebo & Unity (T014-T052)
   - Author B: Module 3 - NVIDIA Isaac (T053-T091)
   - Author C: Module 4 - VLA (T092-T129)
   - Integrator: Coordinates, reviews, validates
3. **Phase 6-7** (parallel integration, 2-3 hours):
   - Author A: Search integration (T130-T138)
   - Author B: Deployment configuration (T139-T152)
   - Author C: Visual assets documentation (T156)
4. **Phase 8**: Team validates together (T153-T165, 2 hours)

**Total Team Time**: 16-20 hours with parallelization

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Run `npm run build` frequently to catch errors early
- Commit after each chapter or logical group (e.g., after each chapter completes)
- Stop at any checkpoint to validate story independently
- Module progression (M2 → M3 → M4) recommended for educational coherence but not technically required
- Search (US4) requires content to index, so best done after US1-US3
- Deployment (US5) requires complete site, so best done last
- All content follows Module 1 quality standards (1500-2500 words, 2-3 diagrams, 3-5 exercises per chapter)
- All Mermaid diagrams must have descriptive alt text for accessibility
- All code examples must be tested (or clearly marked as conceptual if testing requires specialized hardware like NVIDIA GPU)
