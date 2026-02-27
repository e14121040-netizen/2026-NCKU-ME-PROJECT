---
description: How to compile and upload Arduino sketches on this Apple Silicon Mac
---

# Arduino Development Workflow

## Critical Reminders

- **Rosetta 2 is already installed** — DO NOT install Homebrew `avr-gcc` or `avrdude` as workarounds.
- **Homebrew avrdude 8.1 has a silent write bug** — it reports "flash written" and "verified" but the program does NOT actually run. NEVER override toolchain paths with `platform.local.txt`.
- **Always use the original Arduino-bundled x86 toolchain** (runs via Rosetta automatically).

## Steps

// turbo
1. Check that `arduino-cli` is installed: `arduino-cli version`

// turbo
2. Identify the board port: `arduino-cli board list`

// turbo
3. Install any needed libraries: `arduino-cli lib install <name>`

// turbo
4. Compile the sketch: `arduino-cli compile --fqbn <FQBN> <sketch_folder>`
   - If encountering LTO cache errors, add `--clean`
   - Common FQBNs: `arduino:avr:uno`, `arduino:avr:nano`, `arduino:avr:mega`

5. Upload (user must close Serial Monitor first!): `arduino-cli upload --fqbn <FQBN> --port <PORT> <sketch_folder>`

// turbo
6. Open Serial Monitor: `arduino-cli monitor --port <PORT> --config baudrate=9600`

## Troubleshooting

- **Upload fails with "Resource busy"** → Serial Monitor is still open, close it first
- **Module not responding after upload** → Upload a simple Blink test first to verify the board and toolchain are working
- **"bad CPU type in executable"** → Rosetta 2 should be installed: `softwareupdate --install-rosetta --agree-to-license`
- **LTO version mismatch** → Clean cache: `arduino-cli compile --clean ...`
