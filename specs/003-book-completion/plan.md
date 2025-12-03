# Implementation Plan: Book Completion - Modules 2-4, Search & Deployment

**Branch**: `003-book-completion` | **Date**: 2025-11-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/003-book-completion/spec.md`

## Summary

Complete the Physical AI & Humanoid Robotics textbook by writing 9 remaining chapters across 3 modules (Module 2: Gazebo & Unity, Module 3: NVIDIA Isaac, Module 4: VLA), integrating search functionality (local or Algolia), and configuring automated GitHub Pages deployment. This is the final content iteration before the RAG chatbot project.

**Primary Goal**: Deliver a complete, production-ready educational textbook covering ROS 2 fundamentals through cutting-edge VLA models, with full search and automated deployment.

**Technical Approach**: Content creation follows the same structure and quality standards as Module 1 (completed in iteration 2). Each of the 9 chapters will contain 1500-2500 words, 2-3 diagrams (Mermaid-based), 3-5 hands-on exercises, and follow progressive complexity principles. Search will use Docusaurus local search plugin (for sites <100 pages) or Algolia DocSearch (if index size requires). Deployment via GitHub Actions workflow triggered on main branch push.

## Technical Context

**Language/Version**: Markdown (documentation), Node.js 18+ (Docusaurus build)
**Primary Dependencies**:
- Docusaurus 3.9.2 (static site generator, already installed)
- @docusaurus/plugin-ideal-image (already installed)
- @docusaurus/theme-mermaid (already installed)
- @easyops-cn/docusaurus-search-local (to be installed for local search) OR
- Algolia DocSearch (alternative if site grows beyond 100 pages)

**Storage**: Git repository (content as Markdown files), Static assets in `static/img/module-{2,3,4}/`
**Testing**:
- Manual content review (technical accuracy, grammar, clarity)
- `npm run build` (validates Mermaid diagrams, broken links, missing images)
- Lighthouse audits (performance, accessibility, SEO)
- Manual testing (responsive design, dark mode, search functionality)

**Target Platform**: Static HTML/CSS/JS deployed to GitHub Pages (https://username.github.io/repo-name/)
**Project Type**: Documentation site (Docusaurus-based static site)
**Performance Goals**:
- Page load time <3 seconds (p95)
- Build time <2 minutes for full site
- Search results <1 second
- Lighthouse Performance score >90

**Constraints**:
- All code examples must use ROS 2 Humble Hawksbill syntax (Ubuntu 22.04 LTS)
- All images must be <500KB each (optimized)
- WCAG 2.1 AA accessibility compliance required
- No breaking changes to existing Module 1 content
- Content must be educational (not marketing or promotional)

**Scale/Scope**:
- 12 total chapters (3 existing + 9 new)
- 18,000-30,000 total words (Module 1 has ~12,700 words)
- 18-27 total diagrams (Module 1 has 8)
- 27-45 total exercises (Module 1 has 13)
- 4 modules total
- Single-page application (SPA) with client-side routing

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitutional Principles Applied**:

### I. Educational Excellence ✅ PASS
- ✅ All chapters explain "why" before "how" (prerequisites → concepts → application)
- ✅ Progressive complexity maintained (Module 2 builds on Module 1, Module 3 on Module 2, etc.)
- ✅ All technical terms defined on first use with glossary links
- ✅ Examples relate to real-world robotics (Gazebo for testing, Isaac for AI, VLA for manipulation)
- ✅ Learning objectives stated upfront in every chapter

**Justification**: Modules 2-4 follow the same educational structure as Module 1 (already validated in iteration 2).

### II. Visual-Heavy Learning ✅ PASS
- ✅ Minimum 2 diagrams per chapter (spec requires 2-3 per chapter, 18-27 total)
- ✅ All images have descriptive alt text (FR-035 requires WCAG AA compliance)
- ✅ Robotics images from `static/img/` folder (organized by module)
- ✅ Code blocks use syntax highlighting (bash, python, xml, yaml)
- ✅ Mermaid diagrams for architecture and workflows

**Justification**: Visual assets plan from Module 1 demonstrates Mermaid-first approach with accessibility.

### III. Progressive Complexity ✅ PASS
- ✅ Module order is fixed: ROS 2 (M1) → Gazebo/Unity (M2) → NVIDIA Isaac (M3) → VLA (M4)
- ✅ Within modules: basics → integration → advanced patterns
- ✅ No forward references without hyperlinks (navigation links at chapter end)
- ✅ Prerequisites explicitly stated at chapter start (FR-031 requires consistent structure)
- ✅ Key Takeaways listed at chapter end (FR-031)

**Justification**: Spec user stories prioritize P1 (Module 2) → P2 (Module 3) → P3 (Module 4), enforcing learning path.

### IV. Fast & Accessible ✅ PASS
- ✅ Page load time <3 seconds (SC-012, performance goal)
- ✅ Images optimized <500KB each (constraint documented)
- ✅ Responsive design 320px-1920px (SC-014)
- ✅ WCAG 2.1 AA compliance verified (SC-013, FR-035)
- ✅ Dark mode available (Docusaurus default theme)
- ✅ Keyboard navigation fully supported (Docusaurus default)

**Justification**: SC-013 requires Lighthouse Accessibility score >90, ensuring WCAG compliance.

### V. Reference-First Design ✅ PASS
- ✅ Search functionality enabled (P4 user story, FR-021 to FR-025)
- ✅ Table of contents in each chapter (Docusaurus auto-generates from headings)
- ✅ Semantic heading hierarchy (FR-031 requires consistent structure)
- ✅ Code examples copy-paste ready (no ellipsis, tested before publication)
- ✅ External documentation links current and verified (quality standard)

**Justification**: SC-009 requires students to find topics in <10 seconds using search.

### VI. Practical & Hands-On ✅ PASS
- ✅ Every concept has runnable code example (FR-007, FR-013, FR-020)
- ✅ Code tested before publication (testing strategy in plan)
- ✅ Exercises included in every chapter (FR-006, FR-014, SC-004)
- ✅ Capstone project guide in Module 4 (constitution requirement, implied in spec)
- ✅ Hardware/software requirements documented (FR-004, FR-005, FR-011, FR-012)

**Justification**: FR-006 requires 3-5 hands-on exercises per chapter, totaling 27-45 exercises.

**Overall Constitution Compliance**: ✅ **ALL GATES PASS** - No violations, no complexity justifications needed.

## Project Structure

### Documentation (this feature)

```text
specs/003-book-completion/
├── spec.md                  # Feature specification (created by /sp.specify)
├── plan.md                  # This file (/sp.plan output)
├── research.md              # Phase 0 output (research findings)
├── checklists/
│   └── requirements.md      # Spec quality validation (created by /sp.specify)
└── tasks.md                 # Phase 2 output (/sp.tasks command)
```

### Content Structure (repository root)

```text
docs/
├── module-1-ros2/           # EXISTING (from iteration 2)
│   ├── ros2-fundamentals.md
│   ├── nodes-topics-services.md
│   └── building-packages-python.md
├── module-2-gazebo-unity/   # NEW (this iteration)
│   ├── intro-gazebo.md      # Chapter 1: Introduction to Gazebo
│   ├── unity-robotics.md    # Chapter 2: Unity for Robotics
│   └── simulation-best-practices.md  # Chapter 3: Simulation Best Practices
├── module-3-isaac/          # NEW (this iteration)
│   ├── intro-isaac.md       # Chapter 1: Introduction to NVIDIA Isaac
│   ├── isaac-sim.md         # Chapter 2: Isaac Sim for Robotics
│   └── isaac-gym-rl.md      # Chapter 3: Reinforcement Learning with Isaac Gym
└── module-4-vla/            # NEW (this iteration)
    ├── intro-vla.md         # Chapter 1: Introduction to VLA Models
    ├── vla-architectures.md # Chapter 2: VLA Architectures (RT-1, RT-2, PaLM-E)
    └── vla-training-deployment.md  # Chapter 3: Training and Deploying VLA Systems

static/
├── img/
│   ├── module-1/            # EXISTING (from iteration 2)
│   │   └── [existing images]
│   ├── module-2/            # NEW (this iteration)
│   │   ├── gazebo-architecture.png
│   │   ├── unity-robotics-hub.png
│   │   └── [additional diagrams as needed]
│   ├── module-3/            # NEW (this iteration)
│   │   ├── isaac-sim-workflow.png
│   │   ├── isaac-gym-rl.png
│   │   └── [additional diagrams as needed]
│   └── module-4/            # NEW (this iteration)
│       ├── vla-pipeline.png
│       ├── rt1-architecture.png
│       ├── rt2-architecture.png
│       └── [additional diagrams as needed]

.github/
└── workflows/
    └── deploy.yml           # NEW (GitHub Actions deployment workflow)

docusaurus.config.js         # UPDATE (add search plugin configuration)
package.json                 # UPDATE (add search plugin dependency)
```

**Structure Decision**: This is a documentation-only project using Docusaurus static site generator. All content is Markdown files in `docs/` with static assets in `static/img/`. No backend, API, or database required. The structure follows Docusaurus conventions established in iteration 1.

## Complexity Tracking

> **No complexity violations - this section is empty**

All constitution principles pass without exceptions. This iteration follows the same patterns as Module 1 (iteration 2), which successfully validated against all 6 constitutional principles.

---

## Phase 0: Outline & Research

**Goal**: Resolve all technical unknowns and document research findings to enable confident Phase 1 design.

### Research Tasks

**R1: Gazebo Simulation Best Practices**
- **Question**: What are the recommended approaches for integrating Gazebo with ROS 2 Humble?
- **Scope**: Research official Gazebo documentation, ROS 2 Gazebo tutorials, and community best practices for:
  - Gazebo Garden vs. Gazebo Fortress (which version for Ubuntu 22.04?)
  - Launch file patterns for Gazebo + ROS 2 integration
  - URDF/SDF model organization and best practices
  - Physics engine configuration (ODE vs. Bullet)
- **Output**: Document recommended Gazebo version, integration patterns, and example workflows in `research.md`

**R2: Unity Robotics Hub Setup**
- **Question**: What is the current best practice for Unity + ROS 2 integration?
- **Scope**: Research Unity Robotics Hub documentation:
  - Unity version compatibility (Unity 2022 LTS recommended?)
  - ROS 2 Humble integration via Unity Robotics Hub
  - URDF Importer for Unity workflow
  - Communication protocols (ROS TCP Connector vs. native ROS 2)
- **Output**: Document Unity version, Robotics Hub setup steps, and integration patterns in `research.md`

**R3: NVIDIA Isaac Installation & Setup**
- **Question**: What are the system requirements and installation steps for Isaac Sim and Isaac Gym?
- **Scope**: Research NVIDIA Isaac documentation:
  - Isaac Sim system requirements (GPU, VRAM, CUDA version)
  - Isaac Sim installation methods (Omniverse Launcher vs. standalone)
  - Isaac Gym installation (separate package or bundled with Isaac Sim?)
  - ROS 2 integration capabilities (Isaac ROS packages)
- **Output**: Document system requirements, installation steps, and ROS 2 integration options in `research.md`

**R4: VLA Model State-of-the-Art**
- **Question**: What are the current production-ready VLA models and their architectures?
- **Scope**: Research recent VLA literature and implementations:
  - RT-1 (Robotics Transformer 1) architecture and capabilities
  - RT-2 (Robotics Transformer 2) improvements over RT-1
  - PaLM-E (Embodied Multimodal Language Model) integration
  - Open-source VLA implementations (if available)
  - Deployment considerations (model size, inference time, hardware requirements)
- **Output**: Document VLA architectures, comparison table, and deployment recommendations in `research.md`

**R5: Search Plugin Selection**
- **Question**: Should we use local search plugin or Algolia DocSearch for this textbook?
- **Scope**: Compare search solutions:
  - @easyops-cn/docusaurus-search-local: Pros (free, offline, easy setup), Cons (limited to ~100 pages)
  - Algolia DocSearch: Pros (fast, scalable, free for open-source docs), Cons (requires approval, external dependency)
  - Current site size: 12 chapters total (within local search limits)
  - Future growth potential: Unlikely to exceed 100 pages
- **Output**: Document search plugin decision with rationale in `research.md`

**R6: GitHub Pages Deployment Workflow**
- **Question**: What is the recommended GitHub Actions workflow for Docusaurus deployment to GitHub Pages?
- **Scope**: Research Docusaurus deployment documentation:
  - GitHub Actions workflow template for Docusaurus
  - Build optimization strategies (caching dependencies, parallel builds)
  - Error handling and rollback strategies
  - Environment variables and secrets configuration
- **Output**: Document deployment workflow structure and configuration in `research.md`

### Research Output: research.md

**Location**: `specs/003-book-completion/research.md`
**Format**:
```markdown
# Research Findings: Book Completion - Modules 2-4, Search & Deployment

## R1: Gazebo Simulation Best Practices

**Decision**: Use Gazebo Garden (latest stable) with ROS 2 Humble integration
**Rationale**:
- Gazebo Garden is the recommended version for ROS 2 Humble (official compatibility)
- Better physics engine (improved ODE, optional Bullet support)
- Modern launch file patterns using Python launch API

**Alternatives Considered**:
- Gazebo Fortress: Older but stable, less feature-rich
- Gazebo Classic (v11): Deprecated, not recommended for new projects

**Integration Patterns**:
- Use `ros_gz_bridge` for ROS 2 ↔ Gazebo communication
- Launch files: Python-based (not XML) for consistency with Module 1
- Model organization: URDF in `description/` package, SDF for Gazebo-specific worlds

**Code Example Pattern**: [to be documented in research.md]

---

## R2: Unity Robotics Hub Setup
[Similar structure for Unity research]

---

## R3: NVIDIA Isaac Installation & Setup
[Similar structure for Isaac research]

---

## R4: VLA Model State-of-the-Art
[Similar structure for VLA research]

---

## R5: Search Plugin Selection
[Similar structure for search decision]

---

## R6: GitHub Pages Deployment Workflow
[Similar structure for deployment workflow]
```

**Execution**: Use Task tool with subagent_type='Explore' to research each topic and consolidate findings into research.md.

---

## Phase 1: Design & Contracts

**Prerequisites**: research.md complete (all NEEDS CLARIFICATION resolved)

### 1.1 Content Outline Design

**Goal**: Create detailed content outlines for all 9 chapters across Modules 2-4.

**Artifact**: `specs/003-book-completion/content-outline.md` (similar to existing `specs/002-module1-ros2-content/content-outline.md`)

**Structure per Chapter**:
```markdown
## Chapter X: [Title]

**File**: docs/module-X/[filename].md
**Target Word Count**: 1500-2500 words (excluding code blocks)
**Reading Time**: 15-25 minutes
**Priority**: [P1, P2, or P3]

### Section Breakdown

| Section | Word Count | Purpose | Key Deliverables |
|---------|------------|---------|------------------|
| Frontmatter & Prerequisites | 50 | Metadata | sidebar_position, prerequisites |
| Introduction | 150 | Chapter overview | Learning objectives |
| Main Section 1 | 400-500 | Core concept | Explanation + diagram |
| Main Section 2 | 400-500 | Integration | Code example + exercise |
| Main Section 3 | 300-400 | Advanced topic | Troubleshooting |
| Hands-On Exercises | 250-300 | Practice | 3-5 exercises |
| Key Takeaways | 100 | Summary | 7 bullet points |
| Navigation Links | 50 | Cross-references | Prev/Next links |

**Total Target**: ~2000 words (mid-range of 1500-2500)

### Visual Assets
1. [Diagram 1 description]
2. [Diagram 2 description]

### Code Examples
1. [Example 1 description]
2. [Example 2 description]
```

**Chapters to Outline**:

**Module 2: Gazebo & Unity**
1. Introduction to Gazebo (installation, architecture, basic simulation)
2. Unity for Robotics (Unity Hub setup, Robotics Hub, URDF import)
3. Simulation Best Practices (testing strategies, performance optimization)

**Module 3: NVIDIA Isaac**
1. Introduction to NVIDIA Isaac (Isaac Sim overview, use cases, installation)
2. Isaac Sim for Robotics (scene creation, robot import, sensor simulation)
3. Reinforcement Learning with Isaac Gym (RL basics, PPO/SAC, training workflow)

**Module 4: VLA**
1. Introduction to VLA Models (what is VLA, use cases, evolution)
2. VLA Architectures (RT-1, RT-2, PaLM-E comparison)
3. Training and Deploying VLA Systems (dataset requirements, fine-tuning, deployment)

### 1.2 Search Configuration Design

**Goal**: Document search plugin configuration and integration steps.

**Artifact**: `specs/003-book-completion/search-config.md`

**Decision** (from research.md R5):
- Use @easyops-cn/docusaurus-search-local (local search)
- Rationale: Site has 12 total chapters (<100 pages), local search is sufficient and offline-capable

**Configuration**:
```javascript
// docusaurus.config.js
module.exports = {
  themes: [
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        hashed: true,
        language: ["en"],
        indexDocs: true,
        indexBlog: false,
        indexPages: false,
        docsRouteBasePath: '/docs',
        highlightSearchTermsOnTargetPage: true,
        searchResultLimits: 8,
        searchResultContextMaxLength: 50,
      },
    ],
  ],
};
```

**Installation**:
```bash
npm install --save @easyops-cn/docusaurus-search-local
```

**Testing**:
- Verify search bar appears in navigation
- Test search for "ROS 2", "Gazebo", "Isaac Sim", "VLA"
- Verify keyword highlighting works
- Confirm results link to correct sections

### 1.3 Deployment Workflow Design

**Goal**: Create GitHub Actions workflow for automated deployment to GitHub Pages.

**Artifact**: `.github/workflows/deploy.yml`

**Workflow Structure**:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build site
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    needs: build
    runs-on: ubuntu-latest
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

**Build Validation**:
- Build must fail on missing images → `npm run build` will error if `![image](/img/missing.png)` not found
- Build must fail on broken Mermaid → Docusaurus validates Mermaid syntax during build
- Build must fail on broken internal links → Use `@docusaurus/plugin-client-redirects` for link checking

**Error Handling**:
- If build fails, deployment job doesn't run (GitHub Actions dependency: `needs: build`)
- Previous working version remains live on GitHub Pages
- GitHub sends email notification on workflow failure

### 1.4 Image Asset Planning

**Goal**: Document all images and diagrams needed for Modules 2-4.

**Artifact**: `specs/003-book-completion/visual-assets-plan.md` (similar to existing `specs/002-module1-ros2-content/visual-assets-plan.md`)

**Visual Asset Requirements** (from spec):
- Module 2: 6-9 diagrams (2-3 per chapter)
- Module 3: 6-9 diagrams (2-3 per chapter)
- Module 4: 6-9 diagrams (2-3 per chapter)
- Total: 18-27 diagrams

**Asset Creation Strategy**:
1. **Mermaid Diagrams** (preferred): Inline in Markdown, version-controlled, accessible
2. **External Images** (when Mermaid insufficient): Store in `static/img/module-{2,3,4}/`, optimize <500KB

**Example Visual Assets**:

**Module 2**:
- Gazebo architecture diagram (Mermaid graph TD: Physics Engine → Scene Graph → Rendering → Sensors)
- Unity Robotics Hub workflow (Mermaid sequence diagram: Unity ↔ ROS TCP Connector ↔ ROS 2)
- Simulation testing workflow (Mermaid flowchart: Unit Tests → Integration Tests → Sim Tests → Real Robot)

**Module 3**:
- Isaac Sim architecture (Mermaid graph: Omniverse → USD → Physics → Sensors → ROS 2)
- Isaac Gym RL training loop (Mermaid sequence: Environment → Agent → Policy Update → Environment)
- Sim-to-real transfer pipeline (Mermaid flowchart: Simulation → Domain Randomization → Transfer → Real Robot)

**Module 4**:
- VLA pipeline (Mermaid flowchart: Image/Language → Vision Encoder → Language Model → Action Decoder → Robot)
- RT-1 architecture (Mermaid graph: Token Learner → Transformer → Action Tokens)
- RT-2 architecture (Mermaid graph: Vision-Language Model → Action Decoder)

### 1.5 Agent Context Update

**Goal**: Update Claude agent context with technologies from this iteration.

**Artifact**: `.claude/context.md` or equivalent agent-specific file

**Execution**: Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`

**New Technologies to Add**:
- Gazebo Garden simulation
- Unity Robotics Hub
- NVIDIA Isaac Sim
- NVIDIA Isaac Gym
- VLA models (RT-1, RT-2, PaLM-E)
- Docusaurus search plugin (@easyops-cn/docusaurus-search-local)
- GitHub Actions deployment workflow

**Preserve**: Existing context from Module 1 (ROS 2 Humble, Docusaurus, Mermaid)

---

## Phase 2: Task Generation

**Prerequisites**: Phase 1 complete (content-outline.md, search-config.md, deployment workflow designed)

**Execution**: Run `/sp.tasks` command to generate task breakdown in `specs/003-book-completion/tasks.md`

**Expected Task Categories** (preview):

1. **Setup Tasks** (5-10 tasks)
   - Install search plugin
   - Create module directories
   - Set up GitHub Actions workflow

2. **Module 2 Content Tasks** (15-25 tasks)
   - Write Chapter 1: Introduction to Gazebo (outline → draft → polish → validate)
   - Write Chapter 2: Unity for Robotics (outline → draft → polish → validate)
   - Write Chapter 3: Simulation Best Practices (outline → draft → polish → validate)
   - Create 6-9 diagrams for Module 2
   - Test all Gazebo/Unity code examples

3. **Module 3 Content Tasks** (15-25 tasks)
   - Write Chapter 1: Introduction to NVIDIA Isaac
   - Write Chapter 2: Isaac Sim for Robotics
   - Write Chapter 3: Reinforcement Learning with Isaac Gym
   - Create 6-9 diagrams for Module 3
   - Test all Isaac code examples

4. **Module 4 Content Tasks** (15-25 tasks)
   - Write Chapter 1: Introduction to VLA Models
   - Write Chapter 2: VLA Architectures (RT-1, RT-2, PaLM-E)
   - Write Chapter 3: Training and Deploying VLA Systems
   - Create 6-9 diagrams for Module 4
   - Document VLA deployment workflows

5. **Search Integration Tasks** (5-10 tasks)
   - Install and configure search plugin
   - Test search indexing
   - Validate search results quality
   - Document search usage for students

6. **Deployment Tasks** (5-10 tasks)
   - Create GitHub Actions workflow
   - Configure GitHub Pages settings
   - Test deployment pipeline
   - Verify live site functionality

7. **Validation Tasks** (10-20 tasks)
   - Word count validation (each chapter 1500-2500 words)
   - Visual assets validation (2-3 diagrams per chapter)
   - Exercise validation (3-5 per chapter)
   - Build validation (no errors)
   - Lighthouse audits (performance, accessibility)
   - Cross-reference validation (all links work)

**Total Estimated Tasks**: 70-110 tasks

**Time Estimate**: 12-15 hours total (from spec assumptions)
- Module 2: 4-5 hours
- Module 3: 4-5 hours
- Module 4: 4-5 hours
- Search: 1 hour
- Deployment: 1 hour
- Validation: 2 hours

---

## Implementation Phases Summary

| Phase | Deliverables | Time Estimate | Status |
|-------|--------------|---------------|--------|
| **Phase 0: Research** | research.md with 6 research findings | 1-2 hours | Pending |
| **Phase 1: Design** | content-outline.md, search-config.md, deploy.yml, visual-assets-plan.md, agent context | 2-3 hours | Pending |
| **Phase 2: Tasks** | tasks.md with 70-110 tasks | 1 hour | Pending (via /sp.tasks) |
| **Phase 3: Implementation** | 9 chapters, 18-27 diagrams, search, deployment | 12-15 hours | Pending (via /sp.implement) |

**Critical Path**:
1. Phase 0 research must complete before Phase 1 design
2. Module 2 must complete before Module 3 (progressive learning)
3. Module 3 must complete before Module 4 (progressive learning)
4. All modules must complete before final deployment validation

**Parallelization Opportunities**:
- All 3 chapters within a module can be written simultaneously (if multiple authors)
- Diagram creation can happen in parallel with chapter writing
- Search integration can happen in parallel with Module 4 writing

---

## Next Steps

1. **Execute Phase 0**: Run Task tool with research agents to populate research.md
2. **Execute Phase 1**: Generate content-outline.md, search-config.md, deploy.yml, visual-assets-plan.md
3. **Run `/sp.tasks`**: Generate detailed task breakdown
4. **Run `/sp.implement`**: Execute all tasks to complete the book

**Ready for**: Phase 0 research execution (all gates pass, plan complete)
