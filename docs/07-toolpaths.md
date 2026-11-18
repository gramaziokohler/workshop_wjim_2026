<p class="step">STEP 07 / 07</p>

# Toolpaths

Group **8** · one beam of the selected junction, clamped on the table, milled by the IRB2600.

![beam → side → compute → t → unreachable](assets/diagrams/toolpath.svg)

![Group 8 in Grasshopper: Toolpaths, Robot and Calibration](assets/shots/gh_fabrication.png)

| input | |
|---|---|
| `beam` | 0, 1, 2 … a beam of the junction |
| `side` | which reference side faces up · empty: the first setup |
| `stepdown` | depth per pass (mm) |
| `compute` | **press it** · nothing heavy runs before |
| `t` | scrub the robot along the path |

![The IRB2600 at the beam on the table](assets/shots/robot.png)

## Read the list

The robot mills from above, so the beam is milled in **setups**: clamped one side up, milled, turned, clamped again.
`toolpaths` lists them, and what is left to the saw or to you:

```text
setup 1/2: side 0 up, 1300 frames  <- shown
    Tenon: tenon from side 0, 30 mm deep, 1300 frames
setup 2/2: side 2 up, 1300 frames
    Tenon: tenon from side 2, the beam flipped, 30 mm deep, 1300 frames
not milled:
    JackRafterCut: saw ...
```

Set `side` to the next setup's side to see it on the table, and run each setup as its own program.
Between setups, push the beam's reference corner back against the stop at the table's origin.
With a `blank`, every setup mills the same blank.

## Before the real robot

!!! warning "Calibration"
    Measure **A** (origin), **B** (along X), **C** (towards Y) on the table with the robot, type them into `calibration`.
    The angle shown should be close to 90°.

!!! warning "Unreachable"
    `Robot` lists frames it can't reach. Fix them first: another `side`, or another beam position.
