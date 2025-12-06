---
id: 1
title: Update project constitution with initial content
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: [documentation,constitution]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

```text
# /sp.constitution

## Project: Physical AI & Humanoid Robotics Course Book
An open, educational, and technically accurate book built with Docusaurus and deployed to GitHub Pages using Spec-Kit Plus and Claude Code.

## Core Principles
- Maximum educational clarity and technical accuracy
- Easily understandable language (no complex or academic-only English)
- All explanations assume only basic programming/AI background
- Every technical claim must be verifiable and cited
- Reproducibility and hands-on focus (code snippets, commands, and setups must work)
- Zero tolerance for plagiarism or uncited content

## Key Standards
- Writing clarity: Flesch-Kincaid Grade Level 10–12 (strict)
- Language style: simple, direct, educational English only
- Citation format: APA style (in-text + references section per chapter)
- Source priority:
  – Minimum 60% peer-reviewed papers, official documentation, or primary sources
  – Remaining 40% may be high-quality blogs, NVIDIA/ROS docs, GitHub repos (only if no better source exists)
- Minimum 80 sources total across the entire book
- All code snippets must be tested and working
- Always generate and edit content using the context7mcp server connected to Claude Code
- Plagiarism check: 0% tolerance (must pass Copyscape/ZeroGPT/Originality.ai before merge)

## Constraints
- Total word count: 60,000–120,000 words (full book)
- Minimum 12 main chapters + appendices
- Format: Docusaurus MDX pages deployed via GitHub Pages
- Every chapter must include:
  – Learning objectives
  – Hands-on exercises or code examples
  – Further reading with APA citations
- Images/diagrams: must have source or be original (CC-BY-4.0 if reused)

## Success Criteria (Testable)
- Flesch-Kincaid Grade Level strictly 10–12 for all chapters
- 100% of technical claims have an inline citation leading to a verifiable source
- Zero plagiarism (passes all checkers at <2% similarity, excluding citations and code)
- All code examples run without error on Ubuntu 22.04 + ROS 2 Humble/Iron + NVIDIA Isaac Sim
- Book successfully builds and deploys to GitHub Pages with no broken links
- All external links and references are archived (via Wayback Machine or similar) when possible

This constitution must be followed for every commit and chapter.
```

## Response snapshot

The project constitution has been successfully created and updated to version `1.0.0`.

**Version Bump Rationale:** This is the initial creation of the constitution.

**Files flagged for manual follow-up to ensure consistency:**
*   `.specify/templates/plan-template.md`
*   `.specify/templates/spec-template.md`
*   `.specify/templates/tasks-template.md`
*   `.specify/templates/commands/*.md`

**Suggested Commit Message:**
```
docs: amend constitution to v1.0.0 (initial creation)
```

## Outcome

- ✅ Impact: Project constitution created and updated.
- 🧪 Tests: N/A
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully created the project constitution and generated the sync impact report. Manual follow-up is required for template consistency.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation failed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
