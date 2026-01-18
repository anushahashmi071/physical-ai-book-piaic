<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: Added 6 specific principles for Physical AI textbook project
- Added sections: Content and Structural Requirements, Development and Writing Standards
- Removed sections: None
- Templates requiring updates: ⚠ pending review of .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics — AI-Native Textbook Constitution

## Core Principles

### Embodied Intelligence
AI must be explained as systems operating in the physical world

### Systems Thinking
Every concept is part of a larger architecture

### Simulation-First
All learning assumes simulation before real hardware

### Clarity Before Complexity
Intuition precedes math and implementation

### Real-World Constraints
Latency, compute limits, sensors, and failure modes are always discussed

### Pedagogical Excellence
Explain the "why" before the "how"; Introduce mental models before formal definitions; Prefer diagrams and flow descriptions over dense equations; Use consistent metaphors (nervous system, brain, body, senses, actions)

## Content and Structural Requirements
No unexplained acronyms; No assumed cross-chapter knowledge; No hardware dependency beyond simulation unless explicitly stated; Code examples must be minimal, illustrative, and non-boilerplate; Each chapter MUST be self-contained and follow the exact structure: Concept Overview (what this is and why it matters), Mental Model (analogy or system intuition), System Architecture (components and data flow), Minimal Example (illustrative code or pseudo-code only), Common Failure Modes (what breaks in practice), Industry Reality (how this appears in real systems)

## Development and Writing Standards
Write in modular, retrievable sections; Avoid long narrative paragraphs without clear semantic boundaries; Each section must stand alone for RAG retrieval; Avoid references like "as discussed earlier"; All factual explanations must be explicit and grounded in text; Sections must be answerable without external sources; Summaries must be precise, non-speculative, and technical; Content must be adaptable to beginner, intermediate, and advanced levels; Avoid fixed difficulty assumptions; Explanations should be rewritable without changing factual meaning; Professional, technical, and instructional tone; Neutral and globally accessible English; Avoid slang, marketing language, or hype

## Scope Boundaries
Focus on ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems; Do not include unrelated AI topics (e.g., pure NLP, web apps, generic ML); Physical AI is the unifying theme

## Governance
All content generated for this project MUST strictly follow this constitution; Target audience: Undergraduate and early-graduate students in Computer Science, AI, or Engineering with basic Python knowledge assumed but no prior robotics or ROS experience assumed; Success Definition: A student who completes this book can conceptually design, simulate, and reason about a humanoid robot controlled by AI agents operating in physical environments

**Version**: 1.1.0 | **Ratified**: 2026-01-17 | **Last Amended**: 2026-01-17