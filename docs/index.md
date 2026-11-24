---
hide:
  - navigation
  - toc
---

<div class="hero" markdown>
![With Joints in Mind poster](assets/poster.png)

<div markdown>
<p class="date">23–27.11.2026 · NCYU</p>

# With Joints in Mind

Design a timber structure. Pick the place where three beams meet.
Write the joint for it in code. Mill it with a robot.
</div>
</div>

## The round trip

<div class="roundtrip" aria-label="The round trip: Grasshopper, the joint editor and the robot">
  <div class="flow-box rt-gh"><b>RHINO · GRASSHOPPER</b><span><code>joinery.gh</code></span><span>decides <b>where</b>: beams, rules, junctions</span><span class="flow-step">01 install · 02 model</span></div>
  <div class="rt-go rt-go1"><span class="to rt-pink">03 export junction</span><span class="back">05 save = update</span></div>
  <div class="flow-box flow-cyan rt-editor"><b>VS CODE · JOINT EDITOR</b><span><code>corner_node.py</code></span><span>decides <b>how</b>: cuts, as BTLx processings</span><span class="flow-step">04 write · 06 parameterize</span></div>
  <div class="rt-go rt-go2"><span class="to rt-yellow">07 toolpaths</span></div>
  <div class="flow-box flow-yellow rt-robot"><b>ROBOT · IRB2600</b><span>6 mm milling bit</span><span>one beam, one side per setup</span></div>
</div>

## Seven steps

<div class="grid cards" markdown>

-   **01 · Install**

    ---
    One Grasshopper file sets up Rhino. One extension sets up VS Code.

    [:octicons-arrow-right-24: Install](01-install.md)

-   **02 · Timber model**

    ---
    Lines → beams → joint rules → model.

    [:octicons-arrow-right-24: Timber model](02-model.md)

-   **03 · Pick a junction**

    ---
    Where 3+ beams meet. Export it to the editor.

    [:octicons-arrow-right-24: Pick a junction](03-junction.md)

-   **04 · Write the joint**

    ---
    A Python class, live in a 3D viewport.

    [:octicons-arrow-right-24: Write the joint](04-joint-editor.md)

-   **05 · Back to Grasshopper**

    ---
    One line, and the joint is in the structure.

    [:octicons-arrow-right-24: Back to Grasshopper](05-back.md)

-   **06 · Parameterize** · *optional*

    ---
    One joint, many situations.

    [:octicons-arrow-right-24: Parameterize](06-parameterize.md)

-   **07 · Toolpaths**

    ---
    From the joint's cuts to the robot.

    [:octicons-arrow-right-24: Toolpaths](07-toolpaths.md)

</div>

## Appendix

<div class="grid cards" markdown>

-   **Processing lab**

    ---
    Try a processing's toolpath on a beam, without Rhino.

    [:octicons-arrow-right-24: Processing lab](a-processing-lab.md)

</div>
