---
id: 1
title: Overall Book Layout Spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: book-layout-structure
branch: 1-book-layout-structure
user: user
command: /sp.specify
labels: [spec, book-layout, docusaurus]
links:
  spec: specs/1-book-layout-structure/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-book-layout-structure/spec.md
  - specs/1-book-layout-structure/checklists/requirements.md
tests:
  - null
---

## Prompt

```text
sp.specify 1. Overall Book Layout and Structure Specification
WHAT the final book must contain and how it must be organized in Docusaurus:

Title: Physical AI & Humanoid Robotics – From Digital Brain to Embodied Intelligence
Landing page: Hero section + one-paragraph course summary + hardware badge (RTX Required)
Sidebar navigation structure (exact order):
Introduction & Why Physical AI Matters
Learning Outcomes
Hardware & Lab Setup Guide
Module 1 – The Robotic Nervous System (ROS 2)
Module 2 – The Digital Twin (Gazebo & Unity)
Module 3 – The AI-Robot Brain (NVIDIA Isaac Platform)
Module 4 – Vision-Language-Action (VLA) & Conversational Robotics
Capstone Project – The Autonomous Humanoid
Weekly Breakdown & Schedule
Assessments & Grading
Appendices (Installation guides, troubleshooting, full hardware tables)
References (master APA bibliography)

Every module chapter (4–7 above) must contain exactly these sections in this order:
Learning Objectives (bullet list)
Core Concepts (simple English explanations)
Key Technologies & Tools
Hands-on Labs / Code Walkthroughs (runnable examples)
Common Pitfalls & Debugging Tips
Quiz / Self-check Questions
Further Reading (APA citations)

Global elements that must appear on every page:
Top banner: “Requires Ubuntu 22.04 + NVIDIA RTX GPU”
Footer: CC-BY-4.0 license + GitHub edit link + “Built with Spec-Kit Plus”
Right sidebar: “On this page” auto-generated TOC
```

## Response snapshot

Generated the `spec.md` for the book layout and structure, and validated it against quality criteria. All checks passed. The feature branch `1-book-layout-structure` has been created.

## Outcome

- ✅ Impact: Spec for book layout and structure created.
- 🧪 Tests: N/A
- 📁 Files: specs/1-book-layout-structure/spec.md, specs/1-book-layout-structure/checklists/requirements.md created.
- 🔁 Next prompts: Next step is `/sp.plan` or `/sp.clarify` if needed.
- 🧠 Reflection: Successfully generated and validated the spec, handling script execution failures gracefully.

## Evaluation notes (flywheel)

- Failure modes observed: Script execution failures (pwsh not found, create-phr.sh not found) required manual steps.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Improve robust error handling for script execution.