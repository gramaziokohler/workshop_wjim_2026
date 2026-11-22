# Joinery

Design a structure in Grasshopper, pick a junction where three or more beams
meet, design its joint in VS Code with the joint editor, and bring it back into
the structure, all the way to the robot.

## The round trip

Open `joinery.gh` (Rhino units: **millimetres**). The canvas reads left to right:

1. **Design**: centerlines of a hyperbolic-paraboloid canopy.
2. **Beams**: one category per kind of beam (`edge`, `rafter`, `purlin`, `post`).
3. **Joint rules**: the ordinary joints, by category and by topology.
4. **Junctions**: every place where 3+ beams meet, labelled in the viewport. Pick one.
5. **To the joint editor**: give it a name, press `export`. Two files appear next to `joinery.gh`:
   - `<name>.fixture.json`: the junction's beams, cropped and centred. Rewritten on every export.
   - `<name>.py`: a joint module to start from. Written once, then it's yours.
6. **Back from the joint editor**: a line `<junction>: <name>` puts your joint in the design.
   Saving the module in VS Code updates Grasshopper. One module can serve several junctions.
7. **Model**: everything together.
8. **Fabrication**: toolpaths for one beam of the junction (6 mm bit), the beam
   clamped on the table, and the IRB2600 following them (`t` scrubs). Nothing
   is computed until you press `compute`, so picking junctions stays instant;
   until you press it, the last path stays, grey.
   `blank` (mm) mills a piece that long instead of the whole beam: the part around the
   junction, cropped at the table's origin, towards the robot, and its end nearest the junction
   pointing away from it (the beam turned end for end if need be). The parts that are cut away are drawn in `offcut`. The toolpaths don't change,
   only where the beam sits.

`corner_node.py` is a worked example: the joint for the four corners.
`hello_joint.gh` is the same round trip without the design or the robot: ten L corners, each turned
another way. Export one, write its joint, then switch `everywhere` on to put it on every pair:
a joint that was modelled for one pair goes wrong on the others, a parametric one doesn't.
`hello_joint.py` is that joint, and the first one to open: an L butt with a milled lap, in seven steps to uncomment.

Working with a coding agent? Point it at `../skills/write-a-joint/SKILL.md` (or copy that folder
into your agent's skills folder, e.g. `.claude/skills/`).

## The files

| | |
|---|---|
| `joinery.gh` | the Grasshopper document |
| `hello_joint.gh` | one joint on ten pairs of beams: modelled or parametric? |
| `junction_lib.py` | finding junctions, exporting them, generating and loading joint modules |
| `joint_module.py.template` | the joint module `junction_lib.py` writes for a newly exported junction |
| `check_joint.py` | runs a joint module against its Fixture outside VS Code and Rhino, with its toolpaths (`uv run python joinery/check_joint.py joinery/corner_node.py`) |
| `toolpath_lib.py` | toolpaths from BTLx processings (`python toolpath_lib.py` checks itself) |
| `robot_lib.py` | the milling cell and the robot's inverse kinematics (`python robot_lib.py` checks itself) |
| `robot/` | the ABB IRB2600-12/1.65 model |

## Setup

Rhino 8, with the COMPAS Timber Grasshopper components installed from the Package Manager.

Open `00_install.ghx` (repo root) once. It builds the `wjim_ncyu2026` environment every
workshop component runs in, and links `wjim_lib` into it from this repo, so a `git pull`
updates it (restart Rhino to pick up changes). Move the repo, and open `00_install.ghx` again.

The joint editor finds `wjim_lib` through `jointEditor.extraPaths` in `.vscode/settings.json`,
so open the repo root as the VS Code folder.

## Limits, for now

- Beams keep their default orientation (`z_vector` empty): the Fixture can't carry it yet.
- Toolpaths exist for `Lap`, `StepJointNotch`, `Pocket`, `Mortise`, `HouseMortise`, `Slot`, `Drilling`, `Marking`, `Text`, `Chamfer`,
  `BirdsMouth`, `FreeContour`, `MillContour`, `Tenon`, `House` and `LogHouseHalfLap` (two setups: from the top, then the beam flipped),
  `Planing` (a setup per planed side) and `JackRafterCut`, `DoubleCut`, `StepJoint`, `LongitudinalCut`, `TriangleCut`,
  `RidgeValleyCut`, `HipValleyRafterNotch` (what they remove is cleared) (see `toolpath_lib.py`).
  Everything else is listed as "no toolpath yet". Inside corners stay round (3 mm),
  and an undercut is left standing, the note says so.
  `../processing_lab` is a small web page to try them out on a beam, with the robot.
- Without a `blank`, the whole beam goes on the table, and what is more than ~500 mm along it is out of reach.
- One setup per beam: processings on another side are listed, not solved.
- Junction numbers follow the design. Change the design a lot, and check the `assign` lines.
