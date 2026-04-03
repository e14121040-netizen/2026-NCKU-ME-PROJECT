# Project Reorganization Design

**Date:** 2026-04-03

## Goal

Reorganize the entire project repository around `專題計畫書2026.pdf` as the authoritative project narrative, while preserving newer and more practical information that already exists in the repository, such as refined subsystem specs, current progress, and developer-facing guidance.

## Design Principles

1. `專題計畫書2026.pdf` is the primary source for formal report structure and proposal-level content.
2. Existing repository documents are secondary sources used to enrich the project with newer technical details, progress tracking, and implementation notes.
3. If PDF content and repository content conflict:
   - Formal report sections follow the PDF-first structure.
   - README and workflow documents keep newer repository details when they better represent current work.
   - Conflicting or incomplete implementation details are labeled clearly instead of being silently merged.
4. Historical and test materials remain available, but they must be clearly separated from the mainline project story.

## Information Architecture

The repository will be organized into three documentation layers:

### 1. Root overview layer

Files:
- `README.md`
- `BOM.md`
- `開發步驟指南.md`

Role:
- Present the project at a glance.
- Give team members a clear entry point.
- Link to subsystem and report materials.
- Summarize current status without duplicating the entire proposal.

### 2. Formal report layer

File:
- `文件/報告/專題計劃書.md`

Role:
- Follow the PDF chapter structure.
- Absorb newer repository details where appropriate.
- Serve as the Markdown source for further report editing.
- Be cleaned for consistency, wording, terminology, and formatting.

### 3. Subsystem layer

Files:
- `取物機器人(八足Jansen)/README.md`
- `取物機器人(八足Jansen)/BOM.md`
- `取物機器人(八足Jansen)/app_inventor/AppInventor開發指南.md`
- `取物機器人(八足Jansen)/app_inventor/App指令對照表.md`
- `運輸機器人(EV3)/README.md`
- `運輸機器人(EV3)/BOM.md`

Role:
- Hold implementation-facing details for each subsystem.
- Distinguish planning architecture from current prototype state.
- Preserve test and legacy references without confusing them with the active design.

## Mainline Project Narrative

The repository should consistently describe the project as a dual-robot intelligent warehousing logistics system:

- A manually operated picking robot based on a Theo Jansen walking platform.
- An autonomous EV3 transport robot that follows lines and unloads parts.

This narrative should be reflected consistently across the root README, the formal proposal, subsystem READMEs, BOM files, and workflow documents.

## State Labels

To avoid contradictions between planning documents and prototype code, the repository will use explicit state labels where needed:

- `目前規劃`: intended architecture or target design
- `目前實作`: verifiable current implementation state
- `歷史參考`: old references, legacy workflows, or previous-year materials
- `待整合 / 待驗證`: content that is planned or partially described but not yet fully aligned

## Conflict Handling

The most important current mismatch is the picking robot control stack:

- Proposal and current docs describe an `ESP32 + ESP32 C3 + ESP-NOW` architecture.
- `取物機器人(八足Jansen)/arduino/walking_robot.ino` still reflects an older `Arduino MEGA + HC-05` style prototype.

This file will not be falsely rewritten as a finished distributed ESP32 production implementation. Instead, it will be reframed as a prototype or transitional control program, and its comments and surrounding documentation will be updated so it no longer conflicts with the main project narrative.

## Preservation Rules

The following materials are preserved and not folded into the active mainline story:

- `運輸機器人(EV3)/ev3_program_test_sunchen0813/`
  - Preserve as a test and reference version.
- `歷屆學長資料(2021屆)/`
  - Preserve as historical reference material.
- Original PDFs and source assets
  - Keep untouched unless explicitly requested otherwise.

Temporary extraction artifacts created for analysis should be removed before completion.

## Scope of Planned Changes

### Root documents

- Rewrite `README.md` as the project entry point.
- Reconcile `BOM.md` with proposal budget and actual purchases.
- Refactor `開發步驟指南.md` into a practical development workflow guide.

### Formal report

- Restructure `文件/報告/專題計劃書.md` to mirror the PDF more closely.
- Preserve improved technical content from the repo where it strengthens the report.
- Edit for consistency, classification, wording, and proofreading.

### Subsystem documentation

- Align picking robot docs with the approved project story.
- Align EV3 docs with the approved mission flow.
- Preserve old App Inventor references but clearly classify them as legacy or partial references.

### Code-facing clarification

- Update comments and framing in key code files where the repository narrative and code headers currently disagree.
- Preserve `運輸機器人(EV3)/ev3_program_test_sunchen0813/` untouched.

## Verification Plan

After implementation, verify the reorganization through:

1. Structure verification
   - Root README, subsystem README files, BOM files, and workflow docs tell a consistent story.
2. Keyword verification
   - Search for terms such as `HC-05`, `Arduino MEGA`, `ESP-NOW`, and `ESP32` to ensure older terminology only appears in intentionally historical or transitional contexts.
3. Code-to-doc verification
   - Ensure the documentation around active code files does not misrepresent their implementation status.

## Expected Outcome

After reorganization:

- The formal report will be easier to revise for submission.
- The root README will become a reliable project entry point.
- Team members will be able to distinguish active materials, prototype materials, test variants, and historical references.
- Contradictory documentation will be significantly reduced.
