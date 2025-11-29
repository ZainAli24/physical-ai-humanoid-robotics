---
id: 003
title: Create Book Foundation Plan
stage: plan
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5
feature: 001-book-foundation
branch: 001-book-foundation
user: ZAIN ALI
command: /sp.plan
labels: ["plan", "book", "foundation", "docusaurus", "architecture"]
links:
  spec: specs/001-book-foundation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-book-foundation/plan.md
tests:
 - none
---

## Prompt

Create implementation plan for Book Iteration 1: Foundation & Structure

SPECIFICATION REFERENCE:
- US1: Book Structure & Navigation
- US2: Custom Branding & Images
- US3: Initial Content (Preface + Intro)

TECHNICAL CONTEXT:
- Language: TypeScript
- Framework: Docusaurus 3.9.2
- Package Manager: npm
- Runtime: Node.js 20+
- Target Platform: GitHub Pages (future)
- Current State: Fresh Docusaurus init with default template

CONSTITUTION CHECK:
✅ Educational Excellence - Focus on clear structure
✅ Visual-Heavy - Using robotics images
✅ Progressive Complexity - Module-based organization
✅ Fast & Accessible - Responsive design, <3s load
✅ Reference-First - Clear navigation structure

PROJECT STRUCTURE:
docs/
├── preface.md                    # Book introduction
├── intro-physical-ai.md          # Introduction chapter
├── module-1-ros2/               # ROS 2 module
│   ├── category.json
│   ├── ros2-intro.md
│   ├── nodes-topics.md
│   └── [placeholders]
├── module-2-gazebo/             # Gazebo & Unity module
├── module-3-isaac/              # NVIDIA Isaac module
└── module-4-vla/                # VLA module

static/img/
├── book_robotics_favicon_and_book_logo_image00.png
├── book_robotic_image01.png
├── book_robotic_image02.png
└── [other robotics images]

docusaurus.config.ts             # Main configuration
sidebars.ts                      # Sidebar structure

IMPLEMENTATION PHASES:

PHASE 1: Configuration & Branding
- Update docusaurus.config.ts with book metadata
- Set title: "Physical AI & Humanoid Robotics"
- Configure favicon and logo
- Enable dark mode
- Update footer with attribution
- Remove default blog (optional)

PHASE 2: Sidebar Structure
- Create sidebars.ts with 4 module sections
- Each module collapsible
- Preface at top level
- Auto-generated navigation from file structure

PHASE 3: Module Folders & Categories
- Create 4 module folders in docs/
- Add category.json to each module with proper labels
- Create placeholder .md files for chapters (minimum 3 per module)

PHASE 4: Content - Preface
- Write preface.md introducing the book
- Explain: What is Physical AI, Why this book, How to use it
- Add 1 robotics image
- Include learning outcomes

PHASE 5: Content - Introduction
- Write intro-physical-ai.md
- Cover: Definition, History, Applications, Future
- Add 2 robotics images
- Include "Prerequisites" and "Key Takeaways" sections

PHASE 6: Image Integration
- Replace all default Docusaurus images
- Use robotics images from static/img/
- Optimize images if > 500KB
- Add alt text to all images

PHASE 7: Testing & Validation
- Run npm start and verify localhost:3000
- Test navigation on all devices (responsive)
- Test dark mode toggle
- Verify all images load
- Check accessibility (keyboard navigation)

TECHNOLOGY DECISIONS:
- Use TypeScript for config files (default Docusaurus)
- Markdown for content (not MDX initially for speed)
- Mermaid for diagrams (later iterations)
- Local search plugin (Iteration 3)

PERFORMANCE OPTIMIZATIONS:
- Lazy load images
- Minimize JavaScript bundles
- Use default Docusaurus optimizations

FILES TO MODIFY:
1. docusaurus.config.ts - Branding, metadata
2. sidebars.ts - Navigation structure
3. docs/preface.md - NEW
4. docs/intro-physical-ai.md - NEW
5. docs/module-1-ros2/category.json - NEW
6. docs/module-2-gazebo/category.json - NEW
7. docs/module-3-isaac/category.json - NEW
8. docs/module-4-vla/category.json - NEW
9. static/img/[use existing robotics images]

RISKS & MITIGATION:
- Risk: Config errors break build
  Mitigation: Test after each config change
- Risk: Images too large, slow load
  Mitigation: Optimize images < 500KB each

DEFINITION OF DONE:
✅ Book structure visible in sidebar
✅ Preface and intro chapters complete
✅ Custom branding applied
✅ All images display correctly
✅ Builds without errors
✅ Responsive on mobile/tablet/desktop
✅ Dark mode works

## Response snapshot

Created comprehensive implementation plan for Book Iteration 1: Foundation & Structure (001-book-foundation)

**7 Implementation Phases:**
1. Configuration & Branding (15 min) - docusaurus.config.ts updates, disable blog
2. Sidebar Structure (10 min) - Auto-generated with _category_.json approach
3. Module Folders & Categories (10 min) - 4 modules, each with 3+ placeholder chapters
4. Content - Preface (15 min) - ~500 words, 1 image, learning outcomes
5. Content - Introduction (20 min) - ~700 words, 2 images, key takeaways
6. Image Integration & Optimization (10 min) - Verify <500KB, add alt text
7. Cleanup & Testing (20 min) - Delete defaults, validate build, Lighthouse audit

**Key Architectural Decisions:**
- Sidebar approach: Auto-generated with `_category_.json` per module (simpler than manual sidebars.ts)
- Dark mode: Use Docusaurus default `colorMode.respectPrefersColorScheme: true` (already in config)
- Content format: Markdown (not MDX) for speed in Iteration 1
- Module structure: 4 folders in docs/ with descriptive names (module-1-ros2, module-2-gazebo, etc.)

**18 Files to Create:**
- 2 complete chapters (preface.md, intro-physical-ai.md)
- 4 category.json files (module metadata)
- 12 placeholder chapters (3 per module × 4 modules)

**Constitution Compliance:**
✅ All 6 principles verified:
- Educational Excellence - Progressive learning, clear objectives
- Visual-Heavy - Minimum 2 images per completed chapter
- Progressive Complexity - Fixed module order: ROS 2 → Gazebo → Isaac → VLA
- Fast & Accessible - <3s load, responsive 320px-1920px, WCAG 2.1 AA
- Reference-First - Semantic headings, auto-generated TOC
- Practical & Hands-On - Deferred to Iteration 2 (intentional scope limit)

**Time Budget:** 90 minutes total (with 10-minute buffer)

**No Research Phase Needed** - All technical decisions already known (Docusaurus patterns well-documented)

## Outcome

- ✅ Impact: Complete implementation plan created at `specs/001-book-foundation/plan.md` with 7 detailed phases, architecture decisions, file structure, testing checklist, and 90-minute time budget breakdown
- 🧪 Tests: Manual validation plan defined (npm start, Lighthouse audit, responsive testing)
- 📁 Files: Created specs/001-book-foundation/plan.md (comprehensive plan document)
- 🔁 Next prompts: `/sp.tasks` - Generate actionable task list from plan; `/sp.implement` - Execute implementation
- 🧠 Reflection: Plan leverages Docusaurus defaults (auto-generated sidebar, built-in dark mode) to minimize configuration overhead while maximizing speed. Auto-generated sidebar with _category_.json approach is simpler than manual sidebars.ts and scales better for future chapter additions. Placeholder chapters provide clear navigation structure while managing user expectations ("Status: Under Development" banners). Constitution compliance verified across all 6 principles with Principle VI (Practical & Hands-On) intentionally deferred to Iteration 2 per spec scope.

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward Docusaurus configuration with well-documented patterns
- Graders run and results (PASS/FAIL): Constitution Check PASSED (all 6 principles satisfied), Time Budget PASSED (90 minutes within 5-hour total constraint)
- Prompt variant (if applicable): User provided detailed 7-phase structure in prompt; plan expanded with rationale, time estimates, and testing checklists
- Next experiment (smallest change to try): Could experiment with manual sidebars.ts vs auto-generated approach in Iteration 2 if customization needs arise
