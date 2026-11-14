from compas.geometry import Plane
from compas_timber.errors import FeatureApplicationError
from compas_timber.fabrication import DovetailMortise
from compas_timber.fabrication import DovetailTenon
from compas_timber.fabrication import Mortise
from compas_timber.fabrication import Tenon
from compas_timber.fabrication import TenonShapeType
from compas_timber.geometry import brep_difference_first
from compas_timber.geometry import brep_union_first

from .btlx import ParametricProcessingParams
from .geometry import brep_edge_indices_along


# mirrors compas_timber's branch `feature/btlx-2.3-processings` (Tenon.apply), remove once it is released
# compas_timber 3.0.0 fillets the first 8 edges of the volume, which fails for any shape but square, the default included
def _tenon_apply(self, geometry, beam):
    """Apply the feature to the beam geometry.

    Parameters
    ----------
    geometry : :class:`compas.geometry.Brep`
        The geometry to be processed.

    beam : :class:`compas_timber.elements.Beam`
        The beam that is milled by this instance.

    Raises
    ------
    :class:`~compas_timber.errors.FeatureApplicationError`
        If the cutting frames do not create a volume that itersects with beam geometry or any step fails.

    Returns
    -------
    :class:`~compas.geometry.Brep`
        The resulting geometry after processing

    """
    # type: (Brep, Beam) -> Brep
    # get cutting plane from params and beam
    try:
        cutting_plane = Plane.from_frame(self.frame_from_params_and_beam(beam))
    except ValueError as e:
        raise FeatureApplicationError(
            None, geometry.transformed(beam.modeltransformation), "Failed to generate cutting frame from parameters and beam: {}".format(str(e))
        )

    cutting_plane.transform(beam.transformation_to_local())

    # trim geometry with cutting plane
    try:
        geometry.trim(cutting_plane)
    except Exception as e:
        raise FeatureApplicationError(
            cutting_plane.transformed(beam.modeltransformation),
            geometry.transformed(beam.modeltransformation),
            "Failed to trim geometry with cutting plane: {}".format(str(e)),
        )
    # get tenon volume from params and beam
    try:
        tenon_volume = self.volume_from_params_and_beam(beam)
    except ValueError as e:
        raise FeatureApplicationError(
            None, geometry.transformed(beam.modeltransformation), "Failed to generate tenon volume from parameters and beam: {}".format(str(e))
        )
    # fillet the edges of the volume based on the shape
    if self.shape != TenonShapeType.SQUARE:
        try:
            # half the width leaves no flat between two fillets, which is the schema's default and what a Brep cannot fillet
            radius = min(self.shape_radius, 0.499 * min(self.length, self.width))
            tenon_volume.fillet(radius, brep_edge_indices_along(tenon_volume, self.frame_from_params_and_beam(beam).normal))
        except Exception as e:
            # tenon_volume is still in model space here (not yet localized below), geometry is local
            raise FeatureApplicationError(
                tenon_volume,
                geometry.transformed(beam.modeltransformation),
                "Failed to fillet the edges of the tenon volume based on the shape: {}".format(str(e)),
            )

    tenon_volume.transform(beam.transformation_to_local())

    # remove any parts of the volume that exceed the beam geometry. Fails silently.
    for frame in beam.ref_sides[:4]:
        try:
            tenon_volume = tenon_volume.trimmed(frame)
        except Exception:
            pass # Fail silently since it won't be possible to trim the tenon if it doesn't exceed the beam geometry.
    # add tenon volume to geometry
    try:
        geometry = brep_union_first(geometry, tenon_volume)
    except Exception as e:
        raise FeatureApplicationError(
            tenon_volume.transformed(beam.modeltransformation),
            geometry.transformed(beam.modeltransformation),
            "Failed to add tenon volume to geometry: {}".format(str(e)),
        )
    return geometry


# mirrors compas_timber's branch `feature/btlx-2.3-processings` (Mortise.apply), remove once it is released
# compas_timber 3.0.0 fillets the first 8 edges of the volume, which fails for any shape but square, the default included
def _mortise_apply(self, geometry, beam):
    """Apply the feature to the beam geometry.

    Parameters
    ----------
    geometry : :class:`compas.geometry.Brep`
        The geometry to be processed.

    beam : :class:`compas_timber.elements.Beam`
        The beam that is milled by this instance.

    Raises
    ------
    :class:`~compas_timber.errors.FeatureApplicationError`
        If the cutting planes do not create a volume that itersects with beam geometry or any step fails.

    Returns
    -------
    :class:`~compas.geometry.Brep`
        The resulting geometry after processing

    """
    # type: (Brep, Beam) -> Brep

    # get mortise volume from params and beam
    try:
        mortise_volume = self.volume_from_params_and_beam(beam)
    except ValueError as e:
        raise FeatureApplicationError(
            None, geometry.transformed(beam.modeltransformation), "Failed to generate mortise volume from parameters and beam: {}".format(str(e))
        )

    # fillet the edges of the mortise volume based on the shape
    if self.shape != TenonShapeType.SQUARE:
        try:
            # half the width leaves no flat between two fillets, which is the schema's default and what a Brep cannot fillet
            radius = min(self.shape_radius, 0.499 * min(self.length, self.width))
            mortise_volume.fillet(radius, brep_edge_indices_along(mortise_volume, self.frame_from_params_and_beam(beam).normal))
        except Exception as e:
            # mortise_volume is still in model space here (not yet localized below), geometry is local
            raise FeatureApplicationError(
                mortise_volume,
                geometry.transformed(beam.modeltransformation),
                "Failed to fillet the edges of the tenon volume based on the shape: {}".format(str(e)),
            )
    # transform mortise volume to local coordinates of the beam
    mortise_volume.transform(beam.transformation_to_local())

    # remove tenon volume to geometry
    try:
        return brep_difference_first(geometry, mortise_volume)
    except Exception as e:
        raise FeatureApplicationError(
            mortise_volume.transformed(beam.modeltransformation),
            geometry.transformed(beam.modeltransformation),
            "Failed to remove mortise volume from geometry: {}".format(str(e)),
        )


class _NestedProcessing(object):
    """Carries the single nested processing of a :class:`House` or a :class:`HouseMortise`, which the schema allows in addition to the parameters of the base class."""

    NESTED_TYPES = ()

    HAS_REFERENCE_PLANE = True
    WRAPS_SUBPROCESSINGS = False

    @property
    def params(self):
        return ParametricProcessingParams(self)

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def scaled(self, factor):
        new_instance = super(_NestedProcessing, self).scaled(factor)
        if self.subprocessings:
            new_instance.subprocessings = [subprocessing.scaled(factor) for subprocessing in self.subprocessings]
        return new_instance

    def __init__(self, *args, subprocessings=None, **kwargs):
        super(_NestedProcessing, self).__init__(*args, **kwargs)
        for subprocessing in subprocessings or []:
            self.add_subprocessing(subprocessing)

    @property
    def __data__(self):
        data = super(_NestedProcessing, self).__data__
        data["subprocessings"] = self.subprocessings
        return data

    def add_subprocessing(self, subprocessing):
        """Add the nested processing.

        Parameters
        ----------
        subprocessing : :class:`~compas_timber.fabrication.BTLxProcessing`
            The nested processing. Its parameters refer to the element, not to this processing.

        Raises
        ------
        ValueError
            If the type of the processing cannot be nested here, or if there already is a nested processing.

        """
        if type(subprocessing) not in self.NESTED_TYPES:
            raise ValueError("A {} can only hold one of {}. Got: {}".format(self.PROCESSING_NAME, self.NESTED_TYPES, type(subprocessing)))
        if self.subprocessings:
            raise ValueError("A {} can only hold a single nested processing.".format(self.PROCESSING_NAME))
        super(_NestedProcessing, self).add_subprocessing(subprocessing)

    def scale(self, factor):
        """Scale the parameters of this processing and of the nested one by a given factor.

        Parameters
        ----------
        factor : float
            The scaling factor.

        """
        super(_NestedProcessing, self).scale(factor)
        for subprocessing in self.subprocessings or []:
            subprocessing.scale(factor)


class House(_NestedProcessing, Tenon):
    """Represents a House feature: a shallow, wide tenon on which a nested tenon can sit.

    Has the same parameters as a :class:`~compas_timber.fabrication.Tenon`, and optionally a nested :class:`~compas_timber.fabrication.Tenon` or
    :class:`~compas_timber.fabrication.DovetailTenon` whose parameters refer to the element. Its `start_x` and `start_depth` place it on the end face of the house.

    Parameters
    ----------
    subprocessings : list(:class:`~compas_timber.fabrication.BTLxProcessing`), optional
        The nested processing.

    """

    PROCESSING_NAME = "House"  # type: ignore
    NESTED_TYPES = (Tenon, DovetailTenon)

    def apply(self, geometry, beam):
        """Apply the house, and the nested processing if there is one, to the beam geometry.

        Parameters
        ----------
        geometry : :class:`~compas_brep.Brep`
            The geometry to be processed.
        beam : :class:`~compas_timber.elements.Beam`
            The beam that is milled by this instance.

        Raises
        ------
        :class:`~compas_timber.errors.FeatureApplicationError`
            If any of the two cannot be applied.

        Returns
        -------
        :class:`~compas_brep.Brep`
            The resulting geometry after processing.

        """
        geometry = _tenon_apply(self, geometry, beam)
        for subprocessing in self.subprocessings or []:
            # a nested plain Tenon gets the fixed apply as well, without patching the 3.0.0 class
            geometry = _tenon_apply(subprocessing, geometry, beam) if type(subprocessing) is Tenon else subprocessing.apply(geometry, beam)
        return geometry


class HouseMortise(_NestedProcessing, Mortise):
    """Represents a HouseMortise feature: a shallow, wide mortise from whose bottom a nested mortise can continue.

    Has the same parameters as a :class:`~compas_timber.fabrication.Mortise`, and optionally a nested :class:`~compas_timber.fabrication.Mortise` or
    :class:`~compas_timber.fabrication.DovetailMortise` whose parameters refer to the element. Its `start_depth` places it on the bottom of the house.

    Parameters
    ----------
    subprocessings : list(:class:`~compas_timber.fabrication.BTLxProcessing`), optional
        The nested processing.

    """

    PROCESSING_NAME = "HouseMortise"  # type: ignore
    NESTED_TYPES = (Mortise, DovetailMortise)

    def apply(self, geometry, beam):
        """Apply the house mortise, and the nested processing if there is one, to the beam geometry.

        Parameters
        ----------
        geometry : :class:`~compas_brep.Brep`
            The geometry to be processed.
        beam : :class:`~compas_timber.elements.Beam`
            The beam that is milled by this instance.

        Raises
        ------
        :class:`~compas_timber.errors.FeatureApplicationError`
            If any of the two cannot be applied.

        Returns
        -------
        :class:`~compas_brep.Brep`
            The resulting geometry after processing.

        """
        geometry = _mortise_apply(self, geometry, beam)
        for subprocessing in self.subprocessings or []:
            if type(subprocessing) is Mortise:
                # Mortise ignores its start_depth, so it digs from the reference side instead. That stays inside of the house, down to the bottom of the nested mortise.
                subprocessing = subprocessing.copy()
                subprocessing.depth += subprocessing.start_depth
                geometry = _mortise_apply(subprocessing, geometry, beam)  # a nested plain Mortise gets the fixed apply as well
            else:
                geometry = subprocessing.apply(geometry, beam)
        return geometry
