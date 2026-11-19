---
name: write-a-joint
description: Write or change a COMPAS Timber joint module (a `Joint` subclass next to a `.fixture.json`) for the joint editor, built from BTLx processings that the workshop robot can mill. Use when the user wants to design, write, fix, parameterize or explain a joint, a junction, a `*.py` exported from Grasshopper's "To the joint editor", or anything in `joinery/`.
---

# Write a joint

The person you work with is an architecture student learning computational timber joinery.
They watch the joint redraw in the joint editor's viewport while you edit the file. The
point is that they understand the joint, not just that it gets written. So work in small
steps they can see, and explain each cut in carpentry terms.

## The two files

- `<name>.py`: the **joint module**, one `Joint` subclass. This is the file you edit.
- `<name>.fixture.json`: the **Fixture**, the beams of the junction it was exported from.
  `role_bindings` maps each constructor argument to a beam, and `params` sets parameters.
  Grasshopper rewrites `beams` and `role_bindings` on every export, so don't edit them.
  You may edit `params`.

`joinery/corner_node.py` is the worked example. Read it before you write anything.

## How it runs

Fixture → `__init__(roles…, params…)` → `add_extensions()` → `add_features()` → each beam applies its processings.

- `add_extensions()`: lengthen a beam's blank where a cut needs material past its centerline end
  (`beam.add_blank_extension(start, end, self.guid)`, with `beam.extension_to_plane(plane)`).
- `add_features()`: create **BTLx processings**, add them to the beams with `beam.add_features(p)`,
  and keep them in `self.features` (start with `self.features = []`).

## Steps

1. **Write it in words first.** The module docstring has to say which beam is cut, which one
   carries, and what each cut is for. If it doesn't, ask the student and write their answer
   in. Don't write code before this exists.
2. **Pick processings** from the table below. Prefer the ones the robot mills. Every cut must be a
   processing: no Brep booleans and no meshes. The geometry is the result of the processings.
3. **Ask, don't type.** Never hard-code a side index, a dimension or a coordinate that
   comes from this one junction. Ask the beams instead:
   - `self.side_facing(beam, other)`: the side of `beam` that faces `other`, as a Frame.
   - `self.side_against(beam, vector)`, `beam_ref_side_incidence_with_vector`: the side that faces against a direction.
   - `beam.ref_sides[i]` (a Frame), `beam.opp_side(i)`, `beam.get_dimensions_relative_to_side(i)` → `(width, height)`.
   - `self.get_beam_direction_towards_joint(beam)`, `self.location`, `beam.centerline`.
4. **Make it visible.** Every plane, frame or line you compute goes through
   `debug.show(geometry, "short label")`. That is how the student sees your reasoning in the
   viewport. `debug.show()` does nothing outside the editor, so leave the calls in.
5. **Parameters.** Put a value the student will want to tune (depth, length, offset) in
   `__init__` as an annotated keyword (`step_depth: float = 15.0`), store it, add it to
   `__data__`, document it in the class docstring, and add it to the Fixture's `params`.
   Keep numbers as literals in the source, because the editor's Knob turns a literal into a slider.
6. **Check it** after every change. Run this from the repo root:
   ```bash
   uv run python joinery/check_joint.py joinery/<name>.py
   ```
   The check runs the same pipeline as the editor and lists each beam's processings and what happens to each one:
   the side it's milled from and how many frames it takes, or `no toolpath for this processing yet`.
   Loop until it prints `OK`. Then tell the student what to look at in the viewport.
7. **One module, several junctions.** A good joint also works at the other junctions it is
   assigned to (`<junction>: <name>` in Grasshopper). If a number only works for this one junction,
   go back to step 3.

## Processings

The robot mills all of these with a 6 mm bit, so inside corners stay round (r = 3 mm) and an
undercut is left standing. Nothing is sawn. The check's note says how each processing is milled.

| Processing | Constructor | How it's milled |
|---|---|---|
| **Cuts** | | |
| `JackRafterCut` | `from_plane_and_beam(plane, beam, ref_side_index)` | what's cut off is cleared, from the side it leans to |
| `DoubleCut` | `from_planes_and_beam(planes, beam)` | same |
| `StepJoint` | `from_plane_and_beam(plane, beam, step_depth, heel_depth, ref_side_index)` | same, often from two sides |
| `LongitudinalCut` | `from_plane_and_beam(plane, beam, start_x, length, depth)` | same |
| `wjim_lib.TriangleCut` | `from_planes_and_element(planes, element, ref_side_index)` | same |
| `wjim_lib.RidgeValleyCut`, `wjim_lib.HipValleyRafterNotch` | `Cls(**btlx_params)`, read the class docstring | same |
| **Pockets and holes** | | |
| `Lap` | `from_plane_and_beam(plane, beam, length, depth, ref_side_index)`, `from_volume_and_beam(volume, beam)` | pocket |
| `Pocket` | `from_volume_and_element(volume, element, ref_side_index=None)` | pocket (convex) |
| `Mortise`, `wjim_lib.HouseMortise` | `from_frame_and_beam(frame, beam, start_depth, length, width, depth, ref_side_index)` | pocket |
| `Slot` | `from_plane_and_beam(plane, beam, depth, thickness)` | pocket |
| `StepJointNotch` | `from_plane_and_beam(plane, beam, step_depth, strut_height, ref_side_index)` | step notch (step shape only) |
| `BirdsMouth` | `from_planes_and_element(planes, element)` | cleared |
| `Drilling` | `from_line_and_element(line, element, diameter)` | helix |
| `FreeContour`, `wjim_lib.MillContour` | `from_polyline_and_element(polyline, element, depth, interior)` | along the contour |
| **Tenons** | | |
| `Tenon` | `from_plane_and_beam(plane, beam, length, width, height, ref_side_index)` | two setups, the beam flipped |
| `wjim_lib.House` | like `Tenon` | two setups, leaves the nested tenon standing |
| **Surface** | | |
| `wjim_lib.Marking` | `from_line_and_element(line, element, ref_side_index)` | engraved |
| `Text` | `Text(**btlx_params)`, read the class docstring | engraved along its strokes |
| `wjim_lib.Chamfer`, `wjim_lib.Planing`, `wjim_lib.LogHouseHalfLap` | `Cls(**btlx_params)`, read the class docstring | a setup per side |

Import the plain names from `compas_timber.fabrication` and the `wjim_lib.` ones from `wjim_lib`.
Anything else in `compas_timber.fabrication` or `wjim_lib` is a valid BTLx processing, but it has no toolpath yet
(the check says "no toolpath for this processing yet"). Tell the student this before you use one.

## Limits of the workshop setup

- One setup per beam: the robot mills from one side. Processings on several sides of one
  beam mean the beam gets turned between setups. That's fine, but say so.
- Beams keep their default orientation (the Fixture has no `z_vector` yet).
- `ref_side_index` is 0–3 for the long sides and 4–5 for the ends. The side you mill from faces up on the table.

## Don't

- Don't rename or reorder the role arguments of `__init__`, because the Fixture binds beams to them by name.
- Don't edit `beams` or `role_bindings` in the Fixture, or the files in `wjim_lib/` and `joinery/*_lib.py`.
- Don't rewrite the whole module in one go. Make one processing per change, so the student can follow it in the viewport.
