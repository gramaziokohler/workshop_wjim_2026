# wjim_lib

The *With Joints in Mind* workshop library: an extension of `compas_timber==3.0.2`, outside of it.

It holds the BTLx 2.3 processings of compas_timber's branch `feature/btlx-2.3-processings` (commit `113e5d34df`),
which aren't released yet, and will hold the toolpath strategies for them. Whatever gets upstreamed can be deleted here.

```python
from wjim_lib import House, WorkshopBTLxWriter, WorkshopBTLxReader
```

Write and read BTLx with `WorkshopBTLxWriter` / `WorkshopBTLxReader`, not the compas_timber ones (see below).

## Layout

| | |
|---|---|
| `src/wjim_lib/btlx.py` | `ParamSpec`, `ParametricProcessing`, `ParametricProcessingParams`, `WorkshopBTLxWriter` |
| `src/wjim_lib/reader.py` | `WorkshopBTLxReader` |
| `src/wjim_lib/geometry.py` | `brep_edge_indices_along` |
| `src/wjim_lib/<processing>.py` | one module per processing, copied from the branch |
| `src/wjim_lib/toolpaths/` | toolpath strategies (to come) |
| `examples/fabrication/` | a minimal example of each processing (need `compas_viewer`) |
| `docs/btlx_processings.md` | what each processing does, with images |
| `tests/` | the branch's tests, pointed at wjim_lib |

## What it works around in compas_timber 3.0.2

- The 3.0.2 `BTLxReader` only knows processings without subclasses, so it forgets `Tenon` and `Mortise` once `House` and `HouseMortise` are imported.
- The 3.0.2 `BTLxWriter` writes version 2.0.0, can't write repeated or wrapped elements, and scales subprocessings twice (or drops them) in a model in meters.
- `Tenon.apply` / `Mortise.apply` fail for any shape but `square`. `House` and `HouseMortise` carry the fix.
- `DualContour.scale()` has no effect. `SawContour` and `MillContour` carry the fix.

Each override says it mirrors the branch and can go once the branch is released.

## Develop

```bash
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -e ".[dev]"
.venv/bin/python -m pytest
```

Python 3.10+ is needed for the OCC Brep backend; on 3.9 the geometry tests are skipped.
