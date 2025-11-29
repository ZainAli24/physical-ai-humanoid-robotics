<!--
SYNC IMPACT REPORT
===================
Version Change: NEW → 1.0.0
Created: 2025-11-29
Type: Initial Constitution

Principles Added:
- I. Educational Excellence
- II. Visual-Heavy Learning
- III. Progressive Complexity
- IV. Fast & Accessible
- V. Reference-First Design
- VI. Practical & Hands-On

Templates Status:
✅ Initial constitution created
⚠ plan-template.md - Review constitution check section
⚠ spec-template.md - Align with quality standards
⚠ tasks-template.md - Ensure testing/quality tasks

Follow-up TODOs:
- None (all placeholders filled)

Commit Message:
docs: create constitution v1.0.0 for Physical AI & Humanoid Robotics book
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Educational Excellence
Student learning and comprehension take priority over technical complexity or comprehensive coverage.

**Rules:**
- Content MUST explain "why" before "how"
- Concepts MUST build progressively (no prerequisite gaps)
- Every technical term MUST be defined on first use
- Examples MUST relate to real-world robotics applications
- Each chapter MUST state learning objectives upfront

**Rationale:** Students have mixed backgrounds (software/hardware). Accessibility ensures maximum learning impact regardless of prior knowledge.

### II. Visual-Heavy Learning
Every complex concept MUST be accompanied by visual aids (diagrams, images, or code examples).

**Rules:**
- Minimum 2 visual elements per chapter
- All images MUST have descriptive alt text
- Use robotics images from `static/img/` folder (book_robotic_image*.png, book_robotics_image*.png)
- Code blocks MUST include syntax highlighting
- Diagrams MUST use Mermaid where applicable

**Rationale:** Robotics is inherently visual. Architecture diagrams and hardware photos clarify abstract concepts faster than text alone.

### III. Progressive Complexity
Chapters MUST follow strict learning pathway from fundamentals to advanced topics.

**Rules:**
- Module order is fixed: ROS 2 → Gazebo/Unity → NVIDIA Isaac → VLA
- Within modules: basics → integration → advanced patterns
- No forward references without hyperlinks to future chapters
- Prerequisites MUST be explicitly stated at chapter start
- "Key Takeaways" MUST be listed at chapter end

**Rationale:** Students cannot understand NVIDIA Isaac without ROS 2 foundation. Enforced progression prevents confusion and dropout.

### IV. Fast & Accessible
Book MUST load quickly and function on all devices with full accessibility.

**Rules:**
- Page load time MUST be < 3 seconds (p95)
- Images MUST be optimized (< 500KB each)
- Responsive design MUST work on 320px to 1920px widths
- WCAG 2.1 AA compliance MUST be verified
- Dark mode MUST be available
- Keyboard navigation MUST be fully supported

**Rationale:** Students may have slow internet or older devices. Accessibility ensures equitable access to education.

### V. Reference-First Design
Book MUST serve both as learning material AND reference documentation.

**Rules:**
- Search functionality MUST be enabled
- Table of contents MUST appear in each chapter
- Heading hierarchy MUST be semantic (H1 → H2 → H3)
- Code examples MUST be copy-paste ready (no ellipsis or pseudo-code)
- External documentation links MUST be current and verified

**Rationale:** Students return to chapters during projects. Quick lookup of commands and APIs is critical for productivity.

### VI. Practical & Hands-On
Theory MUST be accompanied by working code examples and exercises.

**Rules:**
- Every concept MUST have runnable code example
- Code MUST be tested before publication (no untested snippets)
- Exercises MUST be included (labeled "Try it yourself")
- Capstone project guide MUST be in Module 4
- Hardware/software requirements MUST be documented

**Rationale:** "Learning by doing" yields deepest understanding. Hands-on practice prepares students for real robotics work.

## Quality Standards

### Content Quality
- Technical accuracy MUST be verified against official documentation
- Code examples MUST be tested and error-free
- Grammar and spelling MUST pass automated checks
- Terminology MUST be consistent throughout all modules
- External links MUST be validated before deployment

### Performance
- Initial page load MUST be < 3 seconds
- Build time MUST be < 2 minutes
- Lighthouse performance score MUST be > 90
- No JavaScript console errors permitted
- Images MUST use lazy loading

### Accessibility
- Heading hierarchy MUST be properly nested
- All images MUST have alt text
- Color contrast MUST meet WCAG AA standards (4.5:1 minimum)
- Keyboard navigation MUST reach all interactive elements
- Screen reader compatibility MUST be verified

### Visual Consistency
- Image dimensions MUST be standardized per context (hero: 1200x630, inline: 800x450)
- Code blocks MUST use consistent syntax highlighting theme
- Spacing MUST follow 8px baseline grid
- Typography MUST use system font stack for performance

## Module Structure

### Module 1: The Robotic Nervous System (ROS 2)
**Chapters:** 4
- ROS 2 Fundamentals
- Nodes, Topics, and Services
- Building ROS 2 Packages with Python
- URDF for Humanoid Robots

### Module 2: The Digital Twin (Gazebo & Unity)
**Chapters:** 4
- Gazebo Simulation Environment
- Physics Simulation (Gravity, Collisions)
- Unity Integration for Visualization
- Sensor Simulation (LiDAR, Cameras, IMU)

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
**Chapters:** 4
- NVIDIA Isaac SDK Overview
- AI-Powered Perception
- Reinforcement Learning for Robot Control
- Sim-to-Real Transfer Techniques

### Module 4: Vision-Language-Action (VLA)
**Chapters:** 4
- Voice-to-Action with OpenAI Whisper
- Cognitive Planning with LLMs
- Multi-modal Interaction (Speech, Gesture, Vision)
- Capstone Project: The Autonomous Humanoid

## Development Workflow

### Content Creation Process
1. Outline chapter structure based on module objectives
2. Write content in Markdown (`.md` files)
3. Add code examples with inline testing notes
4. Insert images and diagrams with alt text
5. Self-review against all 6 core principles
6. Test all code examples in target environment
7. Commit to Git with descriptive message

### Quality Gates
- **Pre-Commit:** No spelling errors, all code tested
- **Pre-Build:** All links valid, images optimized
- **Pre-Deploy:** Lighthouse score > 90, accessibility audit passed

### Deployment
- Build MUST complete without errors
- All pages MUST load correctly in preview
- Search index MUST be updated
- Deploy to GitHub Pages using `npm run deploy`
- Verify live URL works and HTTPS is enabled

## Governance

### Constitution Authority
This constitution is the **highest authority** for all book development decisions. When conflicts arise:
1. Constitution overrides personal preferences
2. Constitution overrides convenience
3. Violations MUST be documented with justification

### Amendment Process
To amend this constitution:
1. Identify specific principle requiring change
2. Document reason for change with evidence
3. Increment version number (semantic versioning)
4. Update "Last Amended" date
5. Create ADR documenting the decision
6. Update all dependent templates

### Version Policy
- **MAJOR** (X.0.0): Breaking changes to principles (removes or redefines core rules)
- **MINOR** (x.Y.0): New principle added or existing principle expanded
- **PATCH** (x.y.Z): Clarifications, typo fixes, non-semantic improvements

### Compliance Review
- All pull requests MUST verify adherence to principles
- Complexity (violations) MUST be justified in PR description
- Constitution review MUST occur at each iteration checkpoint

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
