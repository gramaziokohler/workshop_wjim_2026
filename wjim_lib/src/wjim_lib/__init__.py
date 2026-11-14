"""Workshop extension of compas_timber 3.0.0: the BTLx 2.3 processings of compas_timber's branch `feature/btlx-2.3-processings`."""

__version__ = "0.1.0"

from .areas import Apertures
from .areas import GlueArea
from .areas import InsulationArea
from .areas import LockoutArea
from .areas import PlaningArea
from .areas import PlasterArea
from .btlx import ParametricProcessing
from .btlx import ParametricProcessingParams
from .btlx import ParamSpec
from .btlx import WorkshopBTLxWriter
from .chamfer import Chamfer
from .chamfer import ChamferExitType
from .containers import Connector
from .containers import ConnectorInfo
from .containers import ConnectorParameter
from .containers import ProcessingGroup
from .containers import Variant
from .containers import VariantParameter
from .contours import CrampAngleRefType
from .contours import CrampContour
from .contours import MillContour
from .contours import NailContour
from .contours import PenContour
from .contours import SawContour
from .contours import ScrewContour
from .contours import SimpleContour
from .dovetail import Dovetail
from .dovetail import DovetailShapeType
from .dovetail import LapExitType
from .dovetail import ProcessSideType
from .free_surface import FreeSurface
from .free_surface import NurbsPatch
from .geometry import brep_edge_indices_along
from .hip_valley_rafter_notch import HipValleyRafterNotch
from .house import House
from .house import HouseMortise
from .japanese_tenon import JapaneseMortise
from .japanese_tenon import JapaneseTenon
from .log_house_front import LogHouseFront
from .log_house_half_lap import LogHouseHalfLap
from .log_house_joint import LogHouseJoint
from .log_house_joint import LogLapPositionType
from .marking import Marking
from .marking import MarkingStyleType
from .pattern_contour import InsertionPattern
from .pattern_contour import InsertionPatterns
from .pattern_contour import NodeContour
from .pattern_contour import NodeContourSegment
from .pattern_contour import PatternContour
from .planing import Planing
from .profile_cambered import ProfileCambered
from .profile_front import ArcShapeType
from .profile_front import PremillType
from .profile_front import ProfileFront
from .profile_head import ProfileArc
from .profile_head import ProfileHead
from .reader import WorkshopBTLxReader
from .ridge_valley_cut import RidgeValleyCut
from .round_arch import RoundArch
from .saw_cut import SawCut
from .saw_cut import SawCutMachiningLimits
from .scarf_joint import ScarfJoint
from .scarf_joint import ScarfShapeType
from .sphere import Sphere
from .triangle_cut import TriangleCut
from .tyrolean_dovetail import Frosch
from .tyrolean_dovetail import Klingschrot
from .tyrolean_dovetail import TyroleanDovetail
from .tyrolean_dovetail import TyroleanDovetailShapeType

__all__ = [
    "ParametricProcessing",
    "ParametricProcessingParams",
    "ParamSpec",
    "WorkshopBTLxWriter",
    "Connector",
    "ConnectorInfo",
    "ConnectorParameter",
    "ProcessingGroup",
    "Variant",
    "VariantParameter",
    "InsertionPattern",
    "InsertionPatterns",
    "NodeContour",
    "NodeContourSegment",
    "PatternContour",
    "RidgeValleyCut",
    "SawCut",
    "SawCutMachiningLimits",
    "TriangleCut",
    "Chamfer",
    "ChamferExitType",
    "Planing",
    "Sphere",
    "Marking",
    "MarkingStyleType",
    "HipValleyRafterNotch",
    "LogHouseHalfLap",
    "LogHouseJoint",
    "LogLapPositionType",
    "LogHouseFront",
    "ProfileFront",
    "ArcShapeType",
    "PremillType",
    "ProfileCambered",
    "RoundArch",
    "ProfileHead",
    "ProfileArc",
    "SawContour",
    "MillContour",
    "NailContour",
    "CrampContour",
    "ScrewContour",
    "PenContour",
    "SimpleContour",
    "CrampAngleRefType",
    "GlueArea",
    "PlaningArea",
    "PlasterArea",
    "InsulationArea",
    "LockoutArea",
    "Apertures",
    "FreeSurface",
    "NurbsPatch",
    "House",
    "HouseMortise",
    "JapaneseTenon",
    "JapaneseMortise",
    "ScarfJoint",
    "ScarfShapeType",
    "Dovetail",
    "LapExitType",
    "ProcessSideType",
    "DovetailShapeType",
    "TyroleanDovetail",
    "Frosch",
    "Klingschrot",
    "TyroleanDovetailShapeType",
    "WorkshopBTLxReader",
    "brep_edge_indices_along",
]
