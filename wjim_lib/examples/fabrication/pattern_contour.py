"""PatternContour: repeats processings at the nodes of a contour, here a drilling. The contour is drawn for this image."""

from compas.geometry import Frame
from compas_timber.elements import Beam
from compas_timber.fabrication import Drilling
from compas_viewer import Viewer

from wjim_lib import InsertionPattern
from wjim_lib import InsertionPatterns
from wjim_lib import NodeContour
from wjim_lib import NodeContourSegment
from wjim_lib import PatternContour

beam = Beam(Frame.worldXY(), length=800.0, width=120.0, height=200.0)
hole = Drilling(diameter=20.0)
contour = NodeContour([100.0, 30.0, 0.0], [NodeContourSegment([700.0, 30.0, 0.0], 100.0), NodeContourSegment([700.0, 90.0, 0.0]), NodeContourSegment([100.0, 90.0, 0.0], 150.0)])
beam.add_feature(PatternContour(insertion_patterns=InsertionPatterns([InsertionPattern(hole)]), contour=contour, ref_side_index=2))

viewer = Viewer()
viewer.scene.add(beam.compute_elementgeometry(include_features=True))
viewer.show()
