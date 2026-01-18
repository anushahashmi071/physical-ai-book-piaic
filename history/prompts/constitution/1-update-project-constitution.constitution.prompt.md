---
id: 1
title: "Update-project-constitution"
stage: "constitution"
date: "2026-01-17"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "user"
command: "sp.constitution"
labels: ["constitution", "governance", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# PHR: Update project constitution

## User Input

```text
Project: Physical AI & Humanoid Robotics — AI‑Native Textbook
Purpose:
Create a unified, AI‑native technical textbook that teaches Physical AI and Humanoid Robotics, designed for interactive learning, retrieval‑augmented generation (RAG), and personalization.
Target Audience:
Undergraduate and early‑graduate students in Computer Science, AI, or Engineering
Basic Python knowledge assumed
No prior robotics or ROS experience assumed
Core Principles:
Embodied Intelligence: AI must be explained as systems operating in the physical world
Systems Thinking: Every concept is part of a larger architecture
Simulation‑First: All learning assumes simulation before real hardware
Clarity Before Complexity: Intuition precedes math and implementation
Real‑World Constraints: Latency, compute limits, sensors, and failure modes are always discussed
Pedagogical Rules:
Explain the "why" before the "how"
Introduce mental models before formal definitions
Prefer diagrams and flow descriptions over dense equations
Use consistent metaphors (nervous system, brain, body, senses, actions)
Chapter Structure (Mandatory):
Each chapter MUST be self‑contained and follow this exact structure:
Concept Overview (what this is and why it matters)
Mental Model (analogy or system intuition)
System Architecture (components and data flow)
Minimal Example (illustrative code or pseudo‑code only)
Common Failure Modes (what breaks in practice)
Industry Reality (how this appears in real systems)
RAG Anchor Summary (concise factual summary for retrieval)
Content Constraints:
No unexplained acronyms
No assumed cross-chapter knowledge
No hardware dependency beyond simulation unless explicitly stated
Code examples must be minimal, illustrative, and non‑boilerplate
AI‑Native Writing Rules:
Write in modular, retrievable sections
Avoid long narrative paragraphs without clear semantic boundaries
Each section must stand alone for RAG retrieval
Avoid references like "as discussed earlier"
RAG Compatibility:
All factual explanations must be explicit and grounded in text
Sections must be answerable without external sources
Summaries must be precise, non‑speculative, and technical
Personalization Readiness:
Content must be adaptable to beginner, intermediate, and advanced levels
Avoid fixed difficulty assumptions
Explanations should be rewritable without changing factual meaning
Language & Tone:
Professional, technical, and instructional
Neutral and globally accessible English
Avoid slang, marketing language, or hype
Scope Boundaries:
Focus on ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision‑Language‑Action systems
Do not include unrelated AI topics (e.g., pure NLP, web apps, generic ML)
Physical AI is the unifying theme
Success Definition:
A student who completes this book can conceptually design, simulate, and reason about a humanoid robot controlled by AI agents operating in physical environments.
All content generated for this project MUST strictly follow this constitution
```

## Response Summary

Updated the project constitution at `.specify/memory/constitution.md` with specific principles and guidelines for the Physical AI & Humanoid Robotics textbook project. The updated constitution includes six core principles (Embodied Intelligence, Systems Thinking, Simulation-First, Clarity Before Complexity, Real-World Constraints, and Pedagogical Excellence), content and structural requirements, development and writing standards, scope boundaries, and governance rules. The version was updated from 1.0.0 to 1.1.0 to reflect the addition of project-specific principles and requirements.

## Outcome

The constitution has been successfully updated to reflect the specific requirements for the Physical AI & Humanoid Robotics textbook project, ensuring all future development work aligns with the established principles and guidelines.