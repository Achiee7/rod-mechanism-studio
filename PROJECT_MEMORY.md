# Rod Mechanism Studio

Read this file before working. After each user prompt and assistant response, record only the durable decision, completed work, or open item. Keep the goal visible and entries short. Never record credentials or private machine paths here.

## Goal

Build a browser studio for analyzing feasible 2D linkage paths, the forces they exert, and the pistons they require. Preserve the Windows calculator’s engineering workflow and Frame Studio’s design and layout. Publish on GitHub.

## Decisions and progress

- 2026-11-23 · The user considers Frame Studio’s design ready to reuse. Use GPT-6.2 Sol agents for implementation. Created this log before starting work.
- Prior Frame Studio decisions carried forward: CAD ribbon, collapsible left tree, editable properties, visible tool feedback, local saves, and a shareable GitHub Pages URL. Its public release uses Achiee7 and GPL-3.0.
- Legacy mechanism priorities: real hinges and anchors, rigid links and weighted bodies, editable loads and pistons, separate design/simulation poses, signed hold force, hand effort, and stroke sizing.
- Rebuilt and browser-tested the 2D studio. Nine engine regressions pass, including Windows hatch parity (324.59 N hold).
- Published under GPL-3.0 at [Achiee7/rod-mechanism-studio](https://github.com/Achiee7/rod-mechanism-studio). [The live calculator](https://achiee7.github.io/rod-mechanism-studio/) deploys from main.
- Added directional box selection and group editing, 0.1×–4× playback, force overlays, and fixed-barrel pistons with reversible mounting.
- Analysis now sweeps a constrained linkage, previews poses, charts/exports forces and sizes pistons. The example has a rigid bracket; older projects preserve their original joints.
- Verified 23 regressions including four-bar closure, hatch stroke stops, force balance and playback. The updated hatch sweep (−22° to 0°) needs 291.2 mm of travel and peaks at 324.59 N push.
- Kept the analysis graph; unreachable samples hold the drawing at its last reachable pose. Both simulation modes enforce installed stroke and barrel geometry.
- Real physics allows backdriving; Forced motion supports reverse travel and marks overloaded pistons. Verified below-rated, backdriven and forced movement, mounting errors, and hard stops.
- Force results now shows total downward load, piston contribution and remaining lift at the named pin. Canvas badges show extra axial push or pull; the recreated example needs 149.6 N push or 149.4 N upward at Pin 2.

- Fixed short-stroke hatch playback in both modes. Real physics uses uniform hinge inertia and pauses at rest. Force results distinguish the 249.6 N axial stop reaction, zero hand force to hold, and 249.3 N upward to release the test hatch.

## Scope for later

- The earlier Frame Studio project also had 3D and team collaboration. This studio targets standalone 2D mechanism work.
