<!--
SYNC IMPACT REPORT
===================
Version Change: 1.0.0 → 1.1.0
Updated: 2025-12-03
Type: Extension for RAG Chatbot Feature

Principles Added:
- VII. Knowledge Grounding & Accuracy
- VIII. Query Resolution & UX
- IX. Privacy & Data Minimization

Rationale:
RAG-powered chatbot requires specific quality standards for AI-generated
responses, user experience, and data handling. Original principles (I-VI)
covered textbook content only. New principles ensure chatbot maintains
educational integrity while protecting user privacy.

Impact Analysis:
- Features affected: 004-rag-chat, 005-text-selection, 006-auth-sessions
- Existing textbook content (Principles I-VI) unchanged
- Quality gates extended with RAG metrics:
  * Retrieval accuracy > 90%
  * Hallucination rate < 5%
  * Response time p95 < 3s
  * Citation accuracy > 98%

Templates Status:
✅ Constitution extended with RAG principles
⚠ spec-template.md - Add RAG behavior section for chatbot features
⚠ plan-template.md - Add RAG architecture subsection
⚠ tasks-template.md - Add RAG testing phase with quality gates

Follow-up TODOs:
- Create feature specs: 004-rag-chat, 005-text-selection, 006-auth-sessions
- Define 50-question eval dataset for hallucination testing
- Update quality gates in pre-deployment checklist
- Create ADR for Gemini vs GPT-4 model selection

Commit Message:
docs: extend constitution to v1.1.0 with RAG chatbot principles (VII-IX)

Adds three new principles for RAG chatbot feature ensuring AI accuracy,
fast UX, and privacy compliance. Extends quality standards with measurable
RAG-specific metrics (retrieval accuracy, hallucination rate, response time).
-->

# Physical AI & Humanoid Robotics - Project Constitution

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

### VII. Knowledge Grounding & Accuracy
All AI-generated chatbot responses MUST be grounded in textbook content with verifiable sources.

**Rules:**
- RAG retrieval MUST use similarity threshold ≥ 0.7 (cosine similarity)
- Every factual claim MUST cite source chapter/section with clickable URL
- Off-topic queries MUST be refused with redirect: "This topic is not covered in the textbook. Try asking about ROS 2, Gazebo, NVIDIA Isaac, or VLA."
- LLM temperature MUST be ≤ 0.3 for factual accuracy
- Hallucination rate MUST be < 5% (measured on 50-question eval dataset)
- Response MUST NOT include information not found in retrieved context
- Source citations MUST include excerpt (50-150 chars) + heading + similarity score

**Rationale:** Students trust the textbook as authoritative source. AI responses must maintain this trust by never inventing information or providing unsourced claims. Measurable accuracy ensures educational integrity.

### VIII. Query Resolution & UX
Chatbot MUST prioritize fast, helpful responses that enhance learning without disrupting study flow.

**Rules:**
- Response time MUST be < 3 seconds (p95) for typical queries
- Streaming MUST deliver first token within 1 second (immediate feedback)
- Source citations MUST be clickable and navigate to exact section with scroll-to-heading
- Mobile users MUST access chatbot via full-screen modal on viewports < 768px
- Text selection "Add to Chat" MUST have 10-character minimum (avoid accidental triggers)
- Text selection MUST NOT activate inside code blocks (`<pre>`, `<code>` elements)
- Error messages MUST be actionable (e.g., "Service unavailable, retry in 1 minute" NOT "Error 500")
- Chatbot widget MUST NOT block book content (z-index properly managed)

**Rationale:** Students use chatbot for quick clarification during study sessions. Delays, poor mobile UX, or unhelpful errors disrupt learning flow and reduce engagement. Fast, precise answers keep students in "flow state."

### IX. Privacy & Data Minimization
User data MUST be protected with minimal collection, secure storage, and user control.

**Rules:**
- Chat history MUST be user-deletable at any time via UI ("Delete All Chats" button)
- Passwords MUST use bcrypt with ≥ 12 rounds (industry standard for slow hashing)
- Sessions MUST expire after 30 days of inactivity (auto-delete stale sessions)
- API keys MUST never be exposed to frontend (server-side only, no client bundles)
- Anonymous chat MUST work without requiring authentication (graceful degradation)
- User messages MUST NOT be logged to external analytics (Vercel function logs excluded)
- GDPR compliance MUST include data deletion endpoint: `DELETE /api/user/data`
- Database queries MUST NOT contain PII in error messages or stack traces

**Rationale:** Educational tools handle sensitive student data (emails, study patterns, questions revealing knowledge gaps). Privacy-first design builds trust, complies with data protection regulations (GDPR, CCPA), and respects student autonomy over personal information.

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

### RAG Chatbot Quality (Principles VII-IX)
- Retrieval accuracy MUST be > 90% on 50-question eval dataset
- Hallucination rate MUST be < 5% (manual review of 20+ responses)
- All source URLs MUST be validated and reachable (no 404 errors)
- Response time p95 MUST be < 3 seconds under 100 concurrent users
- Citation accuracy MUST be > 98% (correct chapter/section references)
- First token latency MUST be < 1 second (streaming enabled)
- Mobile chatbot MUST be tested on 320px and 375px viewports
- Anonymous chat MUST work without authentication (graceful degradation)
- Password hashing MUST use bcrypt with ≥ 12 rounds
- Session tokens MUST be httpOnly cookies with 30-day expiry

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

**Version**: 1.1.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-12-03

---

## Changelog

### Version 1.1.0 (2025-12-03)
**Type**: MINOR - New principles added for RAG chatbot feature

**Added:**
- Principle VII: Knowledge Grounding & Accuracy
- Principle VIII: Query Resolution & UX
- Principle IX: Privacy & Data Minimization
- RAG Chatbot Quality section in Quality Standards

**Rationale:**
RAG-powered chatbot requires specific standards for AI-generated responses, user experience, and data privacy that were not covered in original textbook-focused principles (I-VI). These additions ensure chatbot maintains educational integrity while protecting student privacy.

**Impact:**
- Applies to features 004-rag-chat, 005-text-selection, 006-auth-sessions
- Original book principles (I-VI) remain unchanged
- Quality gates extended with RAG-specific metrics

### Version 1.0.0 (2025-11-29)
**Type**: MAJOR - Initial constitution

**Added:**
- Principle I: Educational Excellence
- Principle II: Visual-Heavy Learning
- Principle III: Progressive Complexity
- Principle IV: Fast & Accessible
- Principle V: Reference-First Design
- Principle VI: Practical & Hands-On
