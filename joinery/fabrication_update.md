# Fabrication update

Your `joinery.gh` gets the calibrated robot cell, `turn`, and a button that saves your program for milling.
Your design, joints and modules stay as they are.

## 1. Copy the files

1. Close Rhino.
2. Keep a copy of your `joinery.gh`.
3. Copy these into your `joinery` folder, replacing the ones with the same name:

| file | goes to | |
|---|---|---|
| `robot_lib.py` | `joinery/` | replace |
| `milling_tool.json` | `joinery/robot/` | replace |
| `program_lib.py` | `joinery/` | new |
| `fabrication_update.gh` | `joinery/` | new |

## 2. Replace the groups in your joinery.gh

1. Open `joinery.gh` and `fabrication_update.gh`.
2. In **your** `joinery.gh`, delete:
   - everything in group **8 · Fabrication** (Toolpaths, Robot, Calibration and their sliders)
   - in group **9 · Preview colours**, everything **except the two called `beams`**
3. In `fabrication_update.gh`: Ctrl+A, Ctrl+C. In your `joinery.gh`: Ctrl+V. Drag it to a free spot.
   (`Cmd` on a Mac.)
4. Connect two wires:
   - **Model**, output `Model` → `model`
   - **Select Junction**, output `selected` → `junction`
5. Save. Close `fabrication_update.gh` without saving.

Check: **Calibration** says `AB to AC: 89.4°`. Don't change the numbers in `calibration`: they are the real table.

## 3. Save your program

1. Pick the junction and `beam`, press `compute`.
2. Check the `setups` list, and for each `side` in it that **Robot** says every frame is reachable. If not: try `turn`, or a `blank`.
3. Type your team and part in `name`, e.g. `team3_post`. Letters, digits, `_` and `-` only.
4. Press `save`. Give us `team3_post.program.json` from your `joinery` folder.

One file per beam. It holds all the setups. Changed the design? `compute` and `save` again.

## If something is off

| you see | do |
|---|---|
| Toolpaths: `press compute` | press it: the design, `beam` or `stepdown` changed |
| Save Program: `compute first` | the same |
| `No module named program_lib` | the file isn't next to `joinery.gh`, or Rhino was open: restart Rhino |
| robot still where it was before | `robot_lib.py` wasn't replaced, or Rhino was open: restart Rhino |
