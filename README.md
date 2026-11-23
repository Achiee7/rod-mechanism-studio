# Rod Mechanism Studio

[Open Rod Mechanism Studio in your browser](https://achiee7.github.io/rod-mechanism-studio/)

Draw a planar linkage and estimate the gas spring or actuator force needed to hold it. The starter project is a 28 kg hatch with a hinge, bracket and gas spring. The layout follows [Frame Studio](https://github.com/Achiee7/frame-studio).

1. In **Model**, pick a component in the left tree or drawing to edit its properties. Use the ribbon to add joints, rods, weighted bodies, motors, pistons and point loads. Selecting a rod or body partway along its length inserts an attachment when placing a part or load.
2. Choose **Dimension**, pick a member or two points, then place its dimension. Hold **Shift** for the aligned length. Use the constraint tools or Ctrl-click two joints or two members to see their available constraints.
3. Select a piston to inspect its required hold force, moment arm and remaining hand effort. **Force results** shows the estimated peak force, raw connection travel and stroke-limited travel. **Use required force** or **Use peak path force** sets the component’s rated force and push/pull direction.
4. Open **Simulation** for a separate copy of the model. Drag a free pin while paused, or use **Run**, **Step**, **Trace paths** and **Reset pose**. Returning to Model restores the design geometry.

## Calculations

Positions are displayed in millimetres, loads in newtons, masses in kilograms and moments in newton-metres. Static sizing balances external moments about a grounded pivot against force along the piston axis. Positive axial force means push, negative means pull. The example gives a 324.59 N push to hold its initial pose. With the configured 420 N spring, holding the hatch requires about 41.96 N downward at the tip.

The path estimate samples a single-pivot sweep from −80° to +80° around the design pose. Raw connection travel includes positions beyond the piston’s limits. Reported force ranges use the admissible samples. Complex multi-link mechanisms can follow a different path. For those, inspect individual poses in Simulation. Dynamic motion uses an iterative planar solver with gravity, actuator and motor loads, rigid-link constraints and piston stroke stops.

## Projects and development

Edits save in the current browser. Use **Main menu → Export project** for a portable JSON copy. **Open project** accepts both browser projects and workspace JSON from the original Windows Rod Mechanism Calculator. **New project** starts a blank sketch. Undo can restore the previous project during the same session.

The app uses native JavaScript modules and static files. For local development, serve this directory with `python -m http.server 8000 --bind 127.0.0.1` and open `http://127.0.0.1:8000/`. Run the regression suite with `npm test` using Node.js 22 or newer.

GPL-3.0-only. See [LICENSE](LICENSE).
