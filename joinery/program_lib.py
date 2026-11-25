"""Programs: one beam, its processings and its toolpaths in a file, from a design to the fabrication document.

The frames are kept in the design's coordinates, not on a table: where the beam is clamped is decided
where it is milled, by that document's calibration. `python program_lib.py` checks itself.
"""

from __future__ import annotations

from types import SimpleNamespace

import compas
from compas_timber.elements import Beam
from toolpath_lib import Toolpath
from toolpath_lib import _unproxified

SUFFIX = ".program.json"


def save(path, beam, toolpaths, near=None, settings=None):
    """Write `beam`, every processing on it, and `toolpaths`. `near` and `settings` (side, blank...) travel along."""
    processings = [_unproxified(f) for f in beam.features]
    index = {id(p): i for i, p in enumerate(processings)}
    missing = [t for t in toolpaths if id(t.processing) not in index]
    if missing:
        raise ValueError("Not a processing of this beam: {}. Compute the toolpaths again.".format(missing[0]))
    # not the Beam itself: serialised, it loses the joints' processings and blank extensions
    data = {
        "beam": {"frame": beam.frame, "length": beam.length, "width": beam.width, "height": beam.height},
        "extensions": list(beam._resolve_blank_extensions()),
        "processings": processings,
        "toolpaths": [{"processing": index[id(t.processing)], "frames": t.frames, "side": t.ref_side_index, "note": t.note} for t in toolpaths],
        "near": near,
        "settings": settings or {},
    }
    compas.json_dump(data, path)


def load(path):
    """What `save` wrote: `beam`, `features` (the processings its toolpaths are for), `toolpaths`, `near` and `settings`."""
    data = compas.json_load(path)
    if not isinstance(data, dict) or not {"beam", "extensions", "processings", "toolpaths"} <= set(data):
        raise ValueError("{} isn't a program. Save it with the Save Program component.".format(path))
    beam = Beam(**data["beam"])
    beam.add_blank_extension(*data["extensions"])
    processings = data["processings"]
    beam.add_features(processings)
    toolpaths = [Toolpath(processings[t["processing"]], t["frames"], t["side"], t["note"]) for t in data["toolpaths"]]
    milled = {t["processing"] for t in data["toolpaths"]}
    features = [p for i, p in enumerate(processings) if i in milled]
    return SimpleNamespace(beam=beam, features=features, toolpaths=toolpaths, near=data.get("near"), settings=data.get("settings") or {})


if __name__ == "__main__":
    # Self-check: a joint's beams, with extensions and proxied processings, come back the same and so do their toolpaths.
    import os
    import tempfile

    from compas.geometry import Line
    from compas.geometry import Point
    from compas_timber.connections import LLapJoint
    from compas_timber.connections import TTenonMortiseJoint
    from compas_timber.model import TimberModel
    from toolpath_lib import toolpaths

    model = TimberModel()
    a, b, c = [
        Beam.from_centerline(Line(Point(*p), Point(*q)), width=60, height=80)
        for p, q in (((0, 0, 0), (900, 0, 0)), ((0, 0, 0), (0, 700, 0)), ((500, 0, 0), (500, 0, 600)))
    ]
    for beam in (a, b, c):
        model.add_element(beam)
    LLapJoint.create(model, a, b)
    TTenonMortiseJoint.create(model, c, a, tenon_shape="square")
    failures = model.process_joinery()
    assert not failures, failures

    def same(p, q):
        return p.point.distance_to_point(q.point) < 1e-3 and p.zaxis.dot(q.zaxis) > 1 - 1e-6 and p.xaxis.dot(q.xaxis) > 1 - 1e-6

    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, "test" + SUFFIX)
        assert any(sum(beam._resolve_blank_extensions()) > 1 for beam in (a, b, c))
        for beam in (a, b, c):
            made = toolpaths(beam)
            save(path, beam, made, near=Point(1, 2, 3), settings={"side": 2})
            program = load(path)
            assert len(program.beam.features) == len(beam.features) > 0 and program.near == Point(1, 2, 3) and program.settings == {"side": 2}
            assert abs(program.beam.blank_length - beam.blank_length) < 1e-6
            assert all(p.distance_to_point(q) < 1e-6 for p, q in zip(program.beam.blank.points, beam.blank.points))
            assert all(same(p, q) for p, q in zip(program.beam.ref_sides, beam.ref_sides))
            for again in (program.toolpaths, toolpaths(program.beam, program.features)):  # as saved, and made again from the processings
                assert [(t.ref_side_index, t.note, len(t.frames)) for t in again] == [(t.ref_side_index, t.note, len(t.frames)) for t in made]
                assert all(same(p, q) for s, t in zip(again, made) for p, q in zip(s.frames, t.frames))
            partial = [t for t in made if t.processing is made[0].processing]  # a program for one processing doesn't mill the others
            save(path, beam, partial)
            assert len(load(path).features) == 1 and len(load(path).beam.features) == len(beam.features)
        compas.json_dump({"frames": []}, path)
        try:
            load(path)
            raise AssertionError("another file was taken for a program")
        except ValueError:
            pass
    print("Programs come back as they were saved.")
