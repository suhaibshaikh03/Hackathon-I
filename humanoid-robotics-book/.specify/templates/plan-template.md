# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION]  
**Primary Dependencies**: [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]  
**Storage**: [if applicable, e.g., PostgreSQL, CoreData, files or N/A]  
**Testing**: [e.g., pytest, XCTest, cargo test or NEEDS CLARIFICATION]  
**Target Platform**: [e.g., Linux server, iOS 15+, WASM or NEEDS CLARIFICATION]
**Project Type**: [single/web/mobile - determines source structure]  
**Performance Goals**: [domain-specific, e.g., 1000 req/s, 10k lines/sec, 60 fps or NEEDS CLARIFICATION]  
**Constraints**: [domain-specific, e.g., <200ms p95, <100MB memory, offline-capable or NEEDS CLARIFICATION]  
**Scale/Scope**: [domain-specific, e.g., 10k users, 1M LOC, 50 screens or NEEDS CLARIFICATION]

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [ ] **Core Principles Adherence**
  - [ ] Educational Clarity & Technical Accuracy: Ensures content is clear, accurate, understandable, cited, reproducible, and plagiarism-free.
  - [ ] Writing Clarity: Flesch-Kincaid Grade Level 10–12.
  - [ ] Language Style: Simple, direct, educational English only.
  - [ ] Citation Format: APA style (in-text + references).
  - [ ] Source Priority: Min 60% peer-reviewed/official docs/primary sources, 40% high-quality blogs/docs/repos.
  - [ ] Minimum Sources: Min 80 sources total across the book.
  - [ ] Code Snippets: All code snippets must be tested and working.
  - [ ] Content Generation: All content generated/edited using educational best practices and technical accuracy standards.
  - [ ] Plagiarism Check: 0% tolerance (passes Copyscape/ZeroGPT/Originality.ai).
- [ ] **Constraints Met**
  - [ ] Word Count: Plan aligns with total word count goals (60,000–120,000 words).
  - [ ] Chapter Structure: Plan includes minimum 12 main chapters + appendices.
  - [ ] Format: Plan for Docusaurus MDX pages deployed via GitHub Pages.
  - [ ] Chapter Contents: Every chapter includes learning objectives, hands-on exercises/code, and further reading with APA citations.
  - [ ] Images/Diagrams: All images/diagrams have source or are original (CC-BY-4.0 if reused).
- [ ] **Success Criteria Considered**
  - [ ] Flesch-Kincaid: Plan accounts for achieving Grade Level 10–12 strictly.
  - [ ] Citations: Plan ensures 100% technical claims have inline citations.
  - [ ] Plagiarism: Plan ensures 0% plagiarism.
  - [ ] Code Examples: Plan includes testing of all code examples on specified environments.
  - [ ] Build/Deploy: Plan considers successful build and deployment to GitHub Pages.
  - [ ] Archived Links: Plan for archiving external links and references.
- [ ] **Governance**
  - [ ] All development aligns with the constitution for every commit and chapter.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
