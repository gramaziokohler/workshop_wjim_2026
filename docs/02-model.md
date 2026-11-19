<p class="step">STEP 02 / 07</p>

# Timber model

Open **`joinery/joinery.gh`**. Rhino units: **millimetres**.

## The canvas reads left to right

<div class="flow" role="list" aria-label="The eight groups of joinery.gh, left to right">
  <div class="flow-box"><span class="flow-n">1</span><b>Design</b><span>centerlines</span><span class="flow-step">STEP 02</span></div>
  <div class="flow-box"><span class="flow-n">2</span><b>Beams</b><span>by category</span><span class="flow-step">STEP 02</span></div>
  <div class="flow-box"><span class="flow-n">3</span><b>Joint rules</b><span class="flow-step">STEP 02</span></div>
  <div class="flow-box flow-pink"><span class="flow-n">4</span><b>Junctions</b><span>3+ beams</span><a class="flow-step" href="../03-junction/">STEP 03</a></div>
  <div class="flow-box flow-pink"><span class="flow-n">5</span><b>To the editor</b><a class="flow-step" href="../03-junction/">STEP 03</a></div>
  <div class="flow-box"><span class="flow-n">6</span><b>Back from the editor</b><a class="flow-step" href="../05-back/">STEP 05</a></div>
  <div class="flow-box"><span class="flow-n">7</span><b>Model</b><span class="flow-step">STEP 02</span></div>
  <div class="flow-box flow-yellow"><span class="flow-n">8</span><b>Fabrication</b><span>toolpaths, robot</span><a class="flow-step" href="../07-toolpaths/">STEP 07</a></div>
</div>

## 1 · Centerlines

A hyperbolic-paraboloid canopy from five sliders: `span_x`, `span_y`, `height_low`, `height_high`, `divisions`.
Drag them. Everything downstream follows.

## 2 · Beams, by category

| category | section (mm) | |
|---|---|---|
| `edge` | 80 × 160 | the four borders |
| `rafter` | 60 × 120 | one direction of the grid |
| `purlin` | 60 × 120 | the other direction |
| `post` | 100 × 100 | down to the ground |

The category is what the joint rules match on.

## 3 · Joint rules

The ordinary joints: where **two** beams meet (L, T, X), by category and by topology.
`rafter → edge`, `purlin → edge`, `crossings`.

## 7 · Model

Beams + rules → a `TimberModel` with all its cuts.

![The canopy in Rhino, junctions labelled, robot in front](assets/shots/model.png)

!!! note "Fresh Start"
    Keep it in front of **Model**. Without it, every re-run cuts each joint twice.

Two beams meeting is solved. **Three** is where it gets interesting → [Pick a junction](03-junction.md).
