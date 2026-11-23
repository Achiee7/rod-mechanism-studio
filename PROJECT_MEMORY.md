# Rod Mechanism Studio

Read this file before working. After each user prompt and assistant response, record only the durable decision, completed work, or open item. Keep the goal visible and entries short. Never record credentials or private machine paths here.

## Goal

Recreate the Windows rod mechanism calculator as a browser app. Preserve its engineering purpose and core mechanism. Match Frame Studio’s design and layout, adapting its controls and workspace for rod mechanisms. Publish on GitHub.

## Decisions and progress

- 2026-11-23 · The user considers Frame Studio’s design ready to reuse. Use GPT-6.2 Sol agents for implementation. Created this log before starting work.
- Prior Frame Studio decisions carried forward: CAD ribbon, collapsible left tree, editable properties, visible tool feedback, local saves, and a shareable GitHub Pages URL. Its public release uses Achiee7 and GPL-3.0.
- Legacy mechanism priorities: real hinges and anchors, rigid links and weighted bodies, editable loads and pistons, separate design/simulation poses, signed hold force, hand effort, and stroke sizing.
- Rebuilt and browser-tested the 2D studio. Nine engine regressions pass, including Windows hatch parity (324.59 N hold). Public repository created at Achiee7/rod-mechanism-studio.

## Open

- Release destination is GitHub Pages from main. The 3D frame workspace and team collaboration belong to the earlier Frame Studio project and are outside the scope of this 2D release.
