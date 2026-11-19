<p class="step">STEP 03 / 07</p>

# Pick a junction

A **junction** is where three or more beams meet.
Group **4** finds them all and labels them in the viewport.

![Junction labels J0–J4 in the Rhino viewport](assets/shots/junction_labels.png)

| | beams | topology |
|---|---|---|
| J0, J1, J3, J4 | `edge_a`, `edge_b`, `post` | Y · the four corners |
| J2 | `post`, `rafter`, `purlin` | K |

## 1 · Select

**Select Junction** → `junction` slider. The chosen one is highlighted, its beams outlined.

![Junctions and Select Junction in Grasshopper](assets/shots/gh_select_junction.png){ width="560" }

## 2 · Export

Group **5** · **Export Junction**:

1. `name` → a Python name, e.g. `corner_node`
2. `reach` → how much of each beam to take along (mm)
3. press **`export`**

<div class="flow flow-down" aria-label="Export writes two files, VS Code opens, saving brings it back">
  <div class="flow-box"><b>Export Junction</b><span><code>name</code> <code>corner_node</code> · <code>reach</code> <code>600</code> · press <b class="rt-pink">export</b></span></div>
  <div class="flow-box flow-pink"><b>two files</b><span><code>corner_node.fixture.json</code>: the beams, cropped and centred. <span class="rt-pink">Rewritten on every export.</span><br><code>corner_node.py</code>: a joint module to start from. <span class="flow-hi">Written once, then it's yours.</span></span></div>
  <div class="flow-box flow-cyan"><b>VS Code</b><span>opens by itself, the viewport in a tab beside the code</span></div>
  <div class="flow-box"><b>Custom Joints</b><span>assign <code>0: corner_node</code> · save in VS Code → Grasshopper re-solves</span></div>
</div>

![Export Junction with name, reach and the export button](assets/shots/gh_export_junction.png){ width="560" }

!!! tip "Export again any time"
    The `.fixture.json` follows the design. Your `.py` is never overwritten.
