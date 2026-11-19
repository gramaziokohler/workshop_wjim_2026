<p class="step">STEP 06 / 07 · OPTIONAL</p>

# Parameterize

Your joint works at one corner. Does it work at all four?

```text
0: corner_node
1: corner_node
3: corner_node
4: corner_node
```

<div class="flow-box flow-cyan flow-source"><b><code>corner_node.py</code></b><span>one module</span></div>
<div class="flow flow-from" aria-label="One module, four corners: J0, J1, J3, J4">
  <div class="flow-box"><svg viewBox="-60 -42 135 92" aria-hidden="true"><line x1="0" y1="0" x2="0" y2="45" stroke="#f8ebe6" stroke-width="7" stroke-linecap="square"/><line x1="0" y1="0" x2="60" y2="-16" stroke="#ffc23d" stroke-width="5"/><line x1="0" y1="0" x2="-45" y2="-22" stroke="#ff5fa8" stroke-width="5"/></svg><b>J0</b></div>
  <div class="flow-box"><svg viewBox="-60 -42 135 92" aria-hidden="true"><line x1="0" y1="0" x2="0" y2="45" stroke="#f8ebe6" stroke-width="7" stroke-linecap="square"/><line x1="0" y1="0" x2="60" y2="16" stroke="#ffc23d" stroke-width="5"/><line x1="0" y1="0" x2="-45" y2="-22" stroke="#ff5fa8" stroke-width="5"/></svg><b>J1</b></div>
  <div class="flow-box"><svg viewBox="-60 -42 135 92" aria-hidden="true"><line x1="0" y1="0" x2="0" y2="45" stroke="#f8ebe6" stroke-width="7" stroke-linecap="square"/><line x1="0" y1="0" x2="60" y2="-16" stroke="#ffc23d" stroke-width="5"/><line x1="0" y1="0" x2="-45" y2="22" stroke="#ff5fa8" stroke-width="5"/></svg><b>J3</b></div>
  <div class="flow-box"><svg viewBox="-60 -42 135 92" aria-hidden="true"><line x1="0" y1="0" x2="0" y2="45" stroke="#f8ebe6" stroke-width="7" stroke-linecap="square"/><line x1="0" y1="0" x2="60" y2="16" stroke="#ffc23d" stroke-width="5"/><line x1="0" y1="0" x2="-45" y2="22" stroke="#ff5fa8" stroke-width="5"/></svg><b>J4</b></div>
</div>
<p class="flow-caption">same code · different angles, lengths, sections · read them from the beams, don't type them</p>

That's the test: **one module, several junctions.** If one of them breaks, something was typed that should have been asked.

## Ask, don't type

=== "modelled"

    ```python
    post_face = self.post.ref_sides[1]  # true at J0 only
    depth = 15.0            # whatever the beam
    ```

=== "parametric"

    ```python
    post_face = self.side_facing(self.post, edge)
    _, height = edge.get_dimensions_relative_to_side(edge_side)
    ```

Utilities to ask with: `beam_ref_side_incidence`, `beam_ref_side_incidence_with_vector`,
`side_facing`, `get_beam_direction_towards_joint`, `extension_to_plane`.

## Parameters

An annotated number in `__init__` is a parameter. The **Fixture** sets it for the editor:

```json
"params": { "step_depth": 15.0 }
```

## A different situation?

Same idea, other values → copy the module under a new name and assign it where it belongs:

```text
0: corner_node
1: corner_node_deep
```
