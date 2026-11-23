# Rod Mechanism Studio

[Open Rod Mechanism Studio in your browser](https://achiee7.github.io/rod-mechanism-studio/)

Draw a planar linkage and estimate the gas spring or actuator force needed to hold it. The starter project is a 28 kg hatch with a hinge, bracket and gas spring. The layout follows [Frame Studio](https://github.com/Achiee7/frame-studio).

1. In **Model**, pick a component in the left tree or drawing to edit its properties. Use the ribbon to add joints, rods, weighted bodies, motors, pistons and point loads. Selecting a rod or body partway along its length inserts an attachment when placing a part or load.
2. Choose **Dimension**, pick a member or two points, then place its dimension. Hold **Shift** for the aligned length. Use the constraint tools or Ctrl-click two joints or two members to see their available constraints.
3. Drag a box from left to right to select enclosed components, joints and loads. Drag right to left to include crossed components. Shift adds to the selection. Move a group or delete it as one undoable edit.
4. In **Simulation**, choose **Real physics** to use the installed piston force. A heavy load can hold a piston in place or drive it backward. **Forced motion** drives through the available travel even when a piston is overloaded. Use **Reverse travel** to explore the return stroke, including from a fully extended starting position. Select the overload warning to see the available and required forces. Use the eye menu to choose force overlays. Playback speed ranges from 0.1× to 4×, and a single step advances 1/60 s.
5. Open **Analysis**, select a grounded drive member, and choose an angle range. Scrub the chart to preview poses and inspect signed forces, connection lengths and hand effort. **Export sweep** downloads the sampled positions and forces. Widen the angle range to see where the installed piston runs out of stroke.
6. Select a piston in **Model** to set its compressed/extended lengths and force or invert its barrel mounting. The barrel keeps its length as the rod slides. The sizing panel checks whether the requested travel fits the modeled single-stage barrel. Stroke limits and a single-direction peak rating can be applied when the sweep supports them.

## Calculations

Positions are displayed in millimetres, loads in newtons, masses in kilograms and moments in newton-metres. Static sizing balances external moments about a grounded pivot against force along the piston axis. Positive axial force means push, negative means pull. The example gives a 324.59 N push to hold its initial pose. With the configured 420 N spring, holding the hatch requires about 41.96 N downward at the tip.

For a one-degree-of-freedom linkage, the path solver follows the assembly from its design pose, enforces rigid lengths, and checks piston stroke as the drive angle changes. The hatch example starts with a −22° to 0° sweep. Its 73 reachable samples require approximately 291.2 mm of travel. General projects start at ±80°. Selecting an unreachable sample keeps its theoretical value on the graph and leaves the drawing at the last reachable pose. An imported design with incompatible piston geometry can be repaired in Model before running either simulation mode.

Both simulation modes enforce the compressed and extended pin-to-pin lengths and the space available inside the barrel. Forced motion prescribes travel and estimates the drive force needed at the displayed pose. A return stroke that needs another drive is marked on the drawing. If the linkage or conflicting piston commands prevent movement, playback stops. The drive mode is saved with the project.

Hold-force sizing uses virtual work to estimate the force required if one piston supplies the holding effort. The manual lift estimate includes all configured pistons. A change of required force direction, an unavailable pose, or a stroke that cannot fit the barrel is reported beside the sizing results. Force extrema are sampled values. The calculated forces at a paused pose are based on static equilibrium; after a simulation step, the display uses dynamic forces. Redundant member load sharing is identified as an estimate.

## Projects and development

Edits save in the current browser. **Save project** also sets a design checkpoint accessible through **Restore saved design**. Use **Main menu → Export project** for a portable JSON copy. **Open project** accepts browser projects and workspace JSON from the Windows calculator. Older hatch examples may contain a second freely hinged rod at the spring mount. If that mount is a rigid bracket, select the mount and change its member type to **Weighted body** or open the updated example.

The app uses native JavaScript modules and static files. For local development, serve this directory with `python -m http.server 8000 --bind 127.0.0.1` and open `http://127.0.0.1:8000/`. Run the regression suite with `npm test` using Node.js 22 or newer.

GPL-3.0-only. See [LICENSE](LICENSE).
