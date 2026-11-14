"""The BTLx 2.3 machinery of compas_timber's branch `feature/btlx-2.3-processings`, rebuilt on the extension points of compas_timber 3.0.0.

Everything here mirrors that branch and can be removed once it is released.
"""

import xml.etree.ElementTree as ET
from collections import OrderedDict
from dataclasses import dataclass

from compas.geometry import Polyhedron
from compas_brep import Brep
from compas_timber.errors import BTLxProcessingError
from compas_timber.errors import FeatureApplicationError
from compas_timber.fabrication import BTLxPart
from compas_timber.fabrication import BTLxProcessing
from compas_timber.fabrication import BTLxWriter
from compas_timber.fabrication.btlx import AttributeSpec
from compas_timber.fabrication.btlx import BTLxProcessingParams
from compas_timber.geometry import oriented_polyhedron


@dataclass
class ParamSpec(AttributeSpec):
    """An :class:`~compas_timber.fabrication.btlx.AttributeSpec` with what :class:`ParametricProcessing` needs to know about a parameter.

    Parameters
    ----------
    python_name : str
        The name of the Python attribute on the processing class.
    type : type
        The type used for deserialization (e.g. ``float``, ``bool``, ``str``).
        Defaults to ``str``.
    default : object, optional
        The default value. Only used by :class:`ParametricProcessing`.
    limits : tuple(float, float), optional
        The inclusive (min, max) range of the value. Only used by :class:`ParametricProcessing`.
    choices : tuple, optional
        The allowed values. Only used by :class:`ParametricProcessing`.
    scales : bool, optional
        True if the value is a length, which changes with the model's units. Only used by :class:`ParametricProcessing`.
    multiple : bool, optional
        True if the element can repeat. The python attribute is then a list.

    """

    default: object = None
    limits: tuple = None  # type: ignore
    choices: tuple = None  # type: ignore
    scales: bool = False
    multiple: bool = False


def _spec_property(spec):
    """Creates the validated property of a :class:`ParamSpec`."""
    attr_name = "_" + spec.python_name

    def getter(self):
        return getattr(self, attr_name)

    def setter(self, value):
        if value is not None:
            if spec.limits and not spec.limits[0] <= value <= spec.limits[1]:
                raise ValueError("{} must be between {} and {}. Got: {}".format(spec.python_name, spec.limits[0], spec.limits[1], value))
            if spec.choices and value not in spec.choices:
                raise ValueError("{} must be one of {}. Got: {}".format(spec.python_name, spec.choices, value))
            if spec.type is bool and not isinstance(value, bool):
                raise ValueError("{} must be a boolean. Got: {}".format(spec.python_name, value))
        setattr(self, attr_name, value)

    return property(getter, setter)


class ParametricProcessingParams(BTLxProcessingParams):
    """The :class:`~compas_timber.fabrication.btlx.BTLxProcessingParams` of the branch: optional reference plane, optional values, integers and repeated elements."""

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    @property
    def header_attributes(self):
        result = super(ParametricProcessingParams, self).header_attributes
        if not self._instance.HAS_REFERENCE_PLANE:
            del result["ReferencePlaneID"]
        return result

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def as_dict(self):
        result = OrderedDict()
        for btlx_name, attr_spec in self.attribute_map.items():
            value = getattr(self._instance, attr_spec.python_name)
            if value is None:  # optional in the schema, and without a default value
                continue
            result[btlx_name] = str(int(value)) if attr_spec.type is int else self._format_value(value)
        return result

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    @staticmethod
    def _format_value(value):
        if isinstance(value, list):
            return [ParametricProcessingParams._format_value(item) for item in value]
        return BTLxProcessingParams._format_value(value)


class ParametricProcessing(BTLxProcessing):
    """Base class for processings which are fully described by the entries of their ``ATTRIBUTE_MAP``.

    Each :class:`ParamSpec` provides a validated property, a keyword argument of ``__init__``, an entry of ``__data__``
    and, if it is a length, scaling. A property defined explicitly on the subclass takes precedence over the generated one.

    The entries of ``ATTRIBUTE_MAP`` must follow the order of the elements in the BTLx schema, which is an `xs:sequence`.

    """

    HAS_REFERENCE_PLANE = True  # False for the processings which the BTLx schema derives from ProcessingBaseType
    WRAPS_SUBPROCESSINGS = False  # True if the BTLx schema nests the subprocessings in a <Processings> element

    def __init_subclass__(cls, **kwargs):
        for spec in cls.__dict__.get("ATTRIBUTE_MAP", {}).values():
            if spec.python_name not in cls.__dict__:
                setattr(cls, spec.python_name, _spec_property(spec))
        super(ParametricProcessing, cls).__init_subclass__(**kwargs)

    def __init__(self, **kwargs):
        values = {spec.python_name: kwargs.pop(spec.python_name, spec.default) for spec in self.ATTRIBUTE_MAP.values()}
        subprocessings = kwargs.pop("subprocessings", None)
        super(ParametricProcessing, self).__init__(**kwargs)
        self.subprocessings = subprocessings
        for name, value in values.items():
            setattr(self, name, value)

    @property
    def __data__(self):
        data = super(ParametricProcessing, self).__data__
        for spec in self.ATTRIBUTE_MAP.values():
            data[spec.python_name] = getattr(self, spec.python_name)
        for name in ("tool_id", "counter_sink", "tool_position"):
            if getattr(self, name) is not None:
                data[name] = getattr(self, name)
        if self.subprocessings:
            data["subprocessings"] = self.subprocessings
        return data

    @property
    def params(self):
        return ParametricProcessingParams(self)

    def scale(self, factor):
        """Scale the length parameters of this processing.

        Parameters
        ----------
        factor : float
            The scaling factor.

        """
        for spec in self.ATTRIBUTE_MAP.values():
            value = getattr(self, spec.python_name)
            if spec.scales and value is not None:
                setattr(self, spec.python_name, value * factor)
        for subprocessing in self.subprocessings or []:
            subprocessing.scale(factor)

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def scaled(self, factor):
        new_instance = super(ParametricProcessing, self).scaled(factor)
        if self.subprocessings:
            new_instance.subprocessings = [subprocessing.scaled(factor) for subprocessing in self.subprocessings]
        return new_instance

    def volumes_from_params_and_element(self, element):
        """The volumes this processing removes from the element.

        Processings which leave the element's volume as it is (e.g. marking, or nailing) have no volumes.

        Parameters
        ----------
        element : :class:`~compas_timber.base.TimberElement`
            The element this processing is applied to.

        Returns
        -------
        list(:class:`~compas.geometry.Polyhedron` or :class:`~compas_brep.Brep`)
            The volumes, in model coordinates.

        """
        return []

    def apply(self, geometry, element):
        """Subtract the volumes of this processing from the element's geometry.

        Parameters
        ----------
        geometry : :class:`~compas_brep.Brep`
            The element's geometry, in the element's local coordinates.
        element : :class:`~compas_timber.base.TimberElement`
            The element this processing is applied to.

        Raises
        ------
        :class:`~compas_timber.errors.FeatureApplicationError`
            If a volume cannot be generated or subtracted.

        Returns
        -------
        :class:`~compas_brep.Brep`
            The resulting geometry.

        """
        try:
            volumes = self.volumes_from_params_and_element(element)
        except Exception as ex:
            raise FeatureApplicationError(None, geometry, "Failed to generate the {} volume: {}".format(self.PROCESSING_NAME, ex))

        for volume in volumes:
            if isinstance(volume, Polyhedron):
                volume = Brep.from_mesh(oriented_polyhedron(volume).to_mesh())
            local_volume = volume.transformed(element.transformation_to_local())
            try:
                # a removal can split the element, which is one solid, so its largest piece is what remains of it
                geometry = max(Brep.from_boolean_difference(geometry, local_volume), key=lambda piece: piece.volume)
            except Exception as ex:
                raise FeatureApplicationError(volume, geometry.transformed(element.modeltransformation), "Failed to subtract the {} volume: {}".format(self.PROCESSING_NAME, ex))

        for subprocessing in self.subprocessings or []:
            geometry = subprocessing.apply(geometry, element)
        return geometry


class WorkshopBTLxWriter(BTLxWriter):
    """A :class:`~compas_timber.fabrication.BTLxWriter` which writes BTLx 2.3, nested and repeated elements, and scales each processing once.

    Serializers registered with ``register_type_serializer`` are shared with :class:`~compas_timber.fabrication.BTLxWriter`.
    """

    FILE_ATTRIBUTES = OrderedDict(
        [
            ("xmlns", "https://www.design2machine.com"),
            ("Version", "2.3.0"),
            ("Language", "en"),
            ("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance"),
            (
                "xsi:schemaLocation",
                "https://www.design2machine.com https://www.design2machine.com/btlx/BTLx_2_3_0.xsd",
            ),
        ]
    )

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def _create_part(self, element, order_num):
        assert self._tolerance
        # create part element
        scale_factor = 1000.0 if self._tolerance.unit == "M" else 1.0
        part = BTLxPart(element, order_num=order_num, scale_factor=scale_factor)

        part_element = ET.Element("Part", part.attr)
        part_element.extend([part.et_transformations, part.et_grain_direction, part.et_reference_side])
        # create processings element for the part if there are any
        if element.features:
            processings_element = ET.Element("Processings")
            for feature in element.features:
                try:
                    # BTLx always uses mm
                    processing = feature.scaled(1000.0) if self._tolerance.unit == "M" else feature
                    processing_element = self._create_processing(processing)
                except ValueError as ex:
                    self._errors.append(BTLxProcessingError("Failed to create processing: {}".format(ex), part, feature))
                else:
                    processings_element.append(processing_element)
            part_element.append(processings_element)
        if element._geometry:
            part_element.append(part.et_shape)
        return part_element

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def _create_processing(self, processing):
        processing_params = processing.params
        params_dict = processing_params.as_dict()

        processing_element = ET.Element(
            processing_params.header_attributes["Name"],
            processing_params.header_attributes,
        )

        for key, value in params_dict.items():
            if isinstance(value, list):
                # repeated element:
                # <Element /><Element />
                processing_element.extend(self._element_from_complex_param(item) for item in value)
                continue

            if isinstance(value, dict):
                # childless element:
                # <Element key1="value1" key2="value2" />
                param = ET.Element(key)
                for sub_key, sub_value in value.items():
                    param.set(sub_key, sub_value)

            elif isinstance(value, str):
                # single value element:
                # <Element>value</Element>
                param = ET.Element(key)
                param.text = value

            else:
                # complex parameter:
                # <Element><SubElement1 /><SubElement2 /></Element>
                param = self._element_from_complex_param(value)
            processing_element.append(param)

        if processing.subprocessings:  # TODO: expose this in Params as well so this logic only interacts with it
            parent = processing_element
            if getattr(processing, "WRAPS_SUBPROCESSINGS", False):  # the processings of compas_timber 3.0.0 lack it
                parent = ET.Element("Processings")
                processing_element.insert(0, parent)
            for subprocessing in processing.subprocessings:
                parent.append(self._create_processing(subprocessing))
        return processing_element
