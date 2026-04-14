# Project Reorganization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reorganize the repository around `專題計畫書2026.pdf` while preserving newer repository details, clarifying legacy materials, and aligning docs with actual implementation state.

**Architecture:** The repository will be updated in layers. Root documents become the project entry point, the Markdown proposal becomes the formal report source, subsystem files become implementation-facing references, and mismatched code comments are reframed so prototype code no longer conflicts with the project narrative.

**Tech Stack:** Markdown, Arduino `.ino`, Pybricks MicroPython, Git, ripgrep

---

### Task 1: Rebuild the root documentation layer

**Files:**
- Modify: `README.md`
- Modify: `BOM.md`
- Modify: `開發步驟指南.md`

**Step 1: Write the revised root-level structure**

Update the three files so they agree on:
- project identity and goals
- document navigation
- current progress framing
- budget summary framing

**Step 2: Run focused review on the rewritten files**

Run:
```bash
sed -n '1,260p' README.md
sed -n '1,260p' BOM.md
sed -n '1,260p' 開發步驟指南.md
```

Expected:
- consistent terminology
- no contradictory control-stack summary
- clear separation between overview and workflow guidance

**Step 3: Commit the root documentation edits**

```bash
git add README.md BOM.md 開發步驟指南.md
git commit -m "docs: reorganize root project documentation"
```

### Task 2: Rewrite the formal proposal Markdown

**Files:**
- Modify: `文件/報告/專題計劃書.md`

**Step 1: Rebuild the document around the PDF chapter order**

Keep the following structure:
- 一、計畫目標及動機
- 二、文獻回顧
- 三、組員工作分配
- 四、進度規劃甘特圖
- 五、設計概念草圖
- 六、總預算規劃
- 七、參考文獻

Merge in newer repo details only where they improve clarity, specificity, or current status tracking.

**Step 2: Run a proofreading pass**

Run:
```bash
sed -n '1,320p' 文件/報告/專題計劃書.md
```

Expected:
- chapter titles mirror the approved structure
- wording is internally consistent
- budget, progress, and subsystem descriptions match the repo narrative

**Step 3: Commit the proposal rewrite**

```bash
git add 文件/報告/專題計劃書.md
git commit -m "docs: rewrite proposal markdown around approved structure"
```

### Task 3: Align subsystem documents and preserve legacy references

**Files:**
- Modify: `取物機器人(八足Jansen)/README.md`
- Modify: `取物機器人(八足Jansen)/BOM.md`
- Modify: `取物機器人(八足Jansen)/app_inventor/AppInventor開發指南.md`
- Modify: `取物機器人(八足Jansen)/app_inventor/App指令對照表.md`
- Modify: `運輸機器人(EV3)/README.md`
- Modify: `運輸機器人(EV3)/BOM.md`
- Preserve unchanged: `運輸機器人(EV3)/ev3_program_test_sunchen0813/`

**Step 1: Update subsystem docs to classify content**

For each subsystem document:
- identify active design
- identify current implementation state
- identify legacy or historical references

For EV3:
- keep the `sunchen0813` test directory intact
- reference it only as a test/reference variant

**Step 2: Review the subsystem docs together**

Run:
```bash
sed -n '1,260p' '取物機器人(八足Jansen)/README.md'
sed -n '1,220p' '取物機器人(八足Jansen)/BOM.md'
sed -n '1,220p' '運輸機器人(EV3)/README.md'
sed -n '1,220p' '運輸機器人(EV3)/BOM.md'
```

Expected:
- active docs do not present legacy communication methods as current
- test directories are preserved and clearly classified
- BOM wording matches the formal proposal and root BOM

**Step 3: Commit the subsystem document updates**

```bash
git add '取物機器人(八足Jansen)/README.md' \
        '取物機器人(八足Jansen)/BOM.md' \
        '取物機器人(八足Jansen)/app_inventor/AppInventor開發指南.md' \
        '取物機器人(八足Jansen)/app_inventor/App指令對照表.md' \
        '運輸機器人(EV3)/README.md' \
        '運輸機器人(EV3)/BOM.md'
git commit -m "docs: align subsystem documentation with project narrative"
```

### Task 4: Reframe mismatched code comments and metadata

**Files:**
- Modify: `取物機器人(八足Jansen)/arduino/walking_robot.ino`
- Modify: `運輸機器人(EV3)/ev3_program/transport_robot.py`

**Step 1: Update file headers and comments only where needed**

For `walking_robot.ino`:
- mark it as a prototype or transitional control program
- remove the implication that it already represents the final ESP32 distributed architecture

For `transport_robot.py`:
- align descriptive comments with current docs and actual port usage

**Step 2: Re-read the affected headers**

Run:
```bash
sed -n '1,120p' '取物機器人(八足Jansen)/arduino/walking_robot.ino'
sed -n '1,120p' '運輸機器人(EV3)/ev3_program/transport_robot.py'
```

Expected:
- comments describe current reality instead of conflicting with approved documentation

**Step 3: Commit the code-comment alignment**

```bash
git add '取物機器人(八足Jansen)/arduino/walking_robot.ino' \
        '運輸機器人(EV3)/ev3_program/transport_robot.py'
git commit -m "docs: align code headers with current implementation state"
```

### Task 5: Run repository consistency verification and cleanup

**Files:**
- Remove: `專題計畫書2026.extracted.txt`
- Review all files changed in Tasks 1-4

**Step 1: Remove temporary analysis artifacts**

Run:
```bash
rm -f 專題計畫書2026.extracted.txt
```

**Step 2: Run keyword verification**

Run:
```bash
rg -n "HC-05|Arduino MEGA|MEGA|ESP-NOW|ESP32" -S README.md BOM.md 開發步驟指南.md \
  文件/報告/專題計劃書.md \
  '取物機器人(八足Jansen)' \
  '運輸機器人(EV3)'
```

Expected:
- legacy keywords appear only in intentionally historical or transitional contexts
- active documentation consistently reflects the approved project story

**Step 3: Review final git status**

Run:
```bash
git status --short
```

Expected:
- only intended project files are modified
- `運輸機器人(EV3)/ev3_program_test_sunchen0813/` remains untouched

**Step 4: Commit the final cleanup**

```bash
git add README.md BOM.md 開發步驟指南.md 文件/報告/專題計劃書.md \
        '取物機器人(八足Jansen)' '運輸機器人(EV3)' \
        docs/plans/2026-04-03-project-reorganization.md
git commit -m "docs: reorganize repository around 2026 proposal"
```
