"""What wjim_lib works around in compas_timber 3.0.0. The first tests are the branch's additions to test_tenon_mortise.py and
test_free_contour.py, pointed at the workshop classes, since the 3.0.0 Tenon, Mortise and DualContour are left as they are."""

import math

import pytest
from compas.geometry import Frame
from compas.geometry import Polyline
from compas.tolerance import Tolerance
from compas_timber.btlx import BTLxReader
from compas_timber.elements import Beam
from compas_timber.fabrication import DualContour
from compas_timber.fabrication import Mortise
from compas_timber.fabrication import Tenon
from compas_timber.model import TimberModel

from wjim_lib import House
from wjim_lib import HouseMortise
from wjim_lib import MillContour
from wjim_lib import SawContour
from wjim_lib import WorkshopBTLxReader
from wjim_lib import WorkshopBTLxWriter


@pytest.mark.parametrize("shape", ["automatic", "radius", "round"])
def test_house_mortise_rounds_only_the_edges_along_its_depth(shape):
    beam = Beam(Frame.worldXY(), 1000.0, 100.0, 200.0)
    beam.add_feature(HouseMortise(start_x=300.0, start_y=50.0, length=80.0, width=40.0, depth=60.0, shape=shape, shape_radius=10.0))

    volume = beam.compute_elementgeometry(include_features=True).volume

    assert not beam.debug_info
    assert 20000000.0 - volume == pytest.approx(80.0 * 40.0 * 60.0 - (4.0 - math.pi) * 10.0**2 * 60.0, abs=1.0)


def test_house_rounds_only_the_edges_along_its_height():
    beam = Beam(Frame.worldXY(), 1000.0, 100.0, 200.0)
    beam.add_feature(House(orientation="end", start_x=900.0, start_y=50.0, length=80.0, width=40.0, height=60.0, shape="radius", shape_radius=10.0))

    volume = beam.compute_elementgeometry(include_features=True).volume

    assert not beam.debug_info
    assert volume == pytest.approx(18000000.0 + 80.0 * 40.0 * 60.0 - (4.0 - math.pi) * 10.0**2 * 60.0, abs=1.0)


@pytest.mark.parametrize("processing", [HouseMortise(start_x=300.0, start_y=50.0), HouseMortise(start_x=300.0, start_y=50.0, subprocessings=[Mortise(start_x=300.0, start_y=50.0)])])
def test_default_shape_with_a_radius_of_half_the_width_is_applied(processing):
    beam = Beam(Frame.worldXY(), 1000.0, 100.0, 200.0)
    beam.add_feature(processing)

    volume = beam.compute_elementgeometry(include_features=True).volume

    assert not beam.debug_info
    assert volume < 20000000.0


@pytest.mark.parametrize("cls", [SawContour, MillContour])
def test_dual_contour_scale_scales_both_contours(cls):
    contour = DualContour(Polyline([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), Polyline([[1.0, 0.0, 1.0], [2.0, 0.0, 1.0]]))
    processing = cls(contour, tool_position="left", counter_sink=False)

    scaled = processing.scaled(1000.0).contour_param_object

    assert scaled.principal_contour[1].x == 2000.0
    assert scaled.associated_contour[1].z == 1000.0
    assert contour.principal_contour[1].x == 2.0


def test_plain_tenon_and_mortise_are_read_back(tmp_path):
    beam = Beam(Frame.worldXY(), 1000.0, 100.0, 200.0)
    beam.add_feature(Tenon(orientation="end", start_x=900.0))
    beam.add_feature(Mortise(start_x=300.0))
    model = TimberModel(tolerance=Tolerance(unit="MM", absolute=1e-3, relative=1e-3))
    model.add_element(beam)
    path = str(tmp_path / "plain.btlx")
    WorkshopBTLxWriter().write(model, path)

    reader = WorkshopBTLxReader()
    features = list(reader.read(path).elements())[0].features

    assert not reader.errors
    assert [type(feature) for feature in features] == [Tenon, Mortise]
    # why the reader is overridden: House(Tenon) and HouseMortise(Mortise) hide their base classes from the 3.0.0 one
    assert "Tenon" not in BTLxReader()._processing_types
