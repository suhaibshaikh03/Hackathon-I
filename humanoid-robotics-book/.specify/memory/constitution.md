<!--
Sync Impact Report:
Version change: None (initial creation) → 1.0.0
Modified principles: None
Added sections: Core Principles, Key Standards, Constraints, Success Criteria (Testable)
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/*.md: ✅ not applicable (files not found)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Course Book Constitution

## Core Principles

### I. Educational Clarity & Technical Accuracy
Maximum educational clarity and technical accuracy. Easily understandable language (no complex or academic-only English). All explanations assume only basic programming/AI background. Every technical claim must be verifiable and cited. Reproducibility and hands-on focus (code snippets, commands, and setups must work). Zero tolerance for plagiarism or uncited content.

### II. Writing Clarity
Flesch-Kincaid Grade Level 10–12 (strict).

### III. Language Style
Simple, direct, educational English only.

### IV. Citation Format
APA style (in-text + references section per chapter).

### V. Source Priority
Minimum 60% peer-reviewed papers, official documentation, or primary sources. Remaining 40% may be high-quality blogs, NVIDIA/ROS docs, GitHub repos (only if no better source exists). Minimum 80 sources total across the entire book.

### VI. Code Snippets
All code snippets must be tested and working.

### VII. Content Generation
Always generate and edit content using educational best practices and technical accuracy standards.

### VIII. Plagiarism Check
0% tolerance (must pass Copyscape/ZeroGPT/Originality.ai before merge).

## Key Standards

### Writing clarity
Flesch-Kincaid Grade Level 10–12 (strict)

### Language style
Simple, direct, educational English only

### Citation format
APA style (in-text + references section per chapter)

### Source priority
Minimum 60% peer-reviewed papers, official documentation, or primary sources. Remaining 40% may be high-quality blogs, NVIDIA/ROS docs, GitHub repos (only if no better source exists).

### Minimum sources
Minimum 80 sources total across the entire book

### Code snippets
All code snippets must be tested and working

### Content generation
Always generate and edit content using educational best practices and technical accuracy standards

### Plagiarism check
0% tolerance (must pass Copyscape/ZeroGPT/Originality.ai before merge).

## Constraints

Total word count: 60,000–120,000 words (full book). Minimum 12 main chapters + appendices. Format: Docusaurus MDX pages deployed via GitHub Pages. Every chapter must include: – Learning objectives – Hands-on exercises or code examples – Further reading with APA citations. Images/diagrams: must have source or be original (CC-BY-4.0 if reused).

## Success Criteria (Testable)

Flesch-Kincaid Grade Level strictly 10–12 for all chapters. 100% of technical claims have an inline citation leading to a verifiable source. Zero plagiarism (passes all checkers at <2% similarity, excluding citations and code). All code examples run without error on Ubuntu 22.04 + ROS 2 Humble/Iron + NVIDIA Isaac Sim. Book successfully builds and deploys to GitHub Pages with no broken links. All external links and references are archived (via Wayback Machine or similar) when possible.

## Governance

This constitution must be followed for every commit and chapter.

**Version**: 1.0.1 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
