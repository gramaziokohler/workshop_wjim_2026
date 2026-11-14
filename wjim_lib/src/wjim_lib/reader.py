from warnings import warn

from compas_timber.btlx import BTLxReader
from compas_timber.errors import BTLxParsingError
from compas_timber.fabrication import BTLxProcessing

from .containers import frame_from_xml
from .pattern_contour import InsertionPattern
from .pattern_contour import InsertionPatterns


def _concrete_processings(cls):
    """Yields the subclasses of `cls` with a processing name. These can have subclasses of their own, e.g. House is a Tenon."""
    for subclass in cls.__subclasses__():
        if isinstance(subclass.PROCESSING_NAME, str):
            yield subclass
        yield from _concrete_processings(subclass)


class WorkshopBTLxReader(BTLxReader):
    """A :class:`~compas_timber.btlx.BTLxReader` which knows the processings of BTLx 2.3, and reads nested and repeated elements.

    The 3.0.0 reader only knows the processings without subclasses, so it forgets :class:`~compas_timber.fabrication.Tenon`
    and :class:`~compas_timber.fabrication.Mortise` once :class:`~wjim_lib.House` and :class:`~wjim_lib.HouseMortise` are imported.
    """

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def __init__(self):
        super(WorkshopBTLxReader, self).__init__()
        self._processing_types = {cls.PROCESSING_NAME: cls for cls in _concrete_processings(BTLxProcessing)}

    # mirrors compas_timber's branch `feature/btlx-2.3-processings`, remove once it is released
    def _parse_processing(self, processing_elem):
        """Parse a single Processing XML element into a BTLxProcessing object."""
        processing_name = processing_elem.tag.split("}")[-1]  # Remove namespace
        processing_class = self._processing_types.get(processing_name)

        if not processing_class:
            raise BTLxParsingError("Unsupported processing type: {}".format(processing_name), processing_type=processing_name)

        # Get the processing's ATTRIBUTE_MAP for child elements
        attribute_map = getattr(processing_class, "ATTRIBUTE_MAP", {})
        header_attribute_map = getattr(processing_class, "HEADER_ATTRIBUTE_MAP", {})
        kwargs = {}

        # Parse XML attributes (header) using HEADER_ATTRIBUTE_MAP
        for xml_attr_name, xml_attr_value in processing_elem.attrib.items():
            if xml_attr_name in header_attribute_map:
                python_name, type_info = header_attribute_map[xml_attr_name]
                kwargs[python_name] = self._convert_value(xml_attr_value, type_info)
            else:
                # Unknown header attribute - skip and warn, processing can still be created
                warn("BTLx: unknown header attribute '{}' for processing type '{}'.".format(xml_attr_name, processing_name), UserWarning, stacklevel=2)

        # Parse child elements using processing's ATTRIBUTE_MAP
        subprocessings = []
        for child in processing_elem:
            child_name = child.tag.split("}")[-1]  # Remove namespace
            if child_name == "InsertionPatterns":
                kwargs["insertion_patterns"] = self._parse_insertion_patterns(child)

            elif child_name in attribute_map:
                attr_spec = attribute_map[child_name]
                # Extract python_name and type_info from AttributeSpec
                python_name = attr_spec.python_name
                type_info = attr_spec.type

                # Branch 0: Type which parses its own element
                if hasattr(type_info, "from_xml"):
                    value = type_info.from_xml(child)
                    if getattr(attr_spec, "multiple", False):  # a 3.0.0 AttributeSpec has no `multiple`
                        kwargs.setdefault(python_name, []).append(value)
                    else:
                        kwargs[python_name] = value

                # Branch 1: Element with attributes → dict
                # <Element key1="value1" key2="value2" />
                elif len(child.attrib) > 0 and not (child.text and child.text.strip()) and len(child) == 0:
                    # Build dict from XML attributes - values stay as strings
                    xml_attr_dict = {xml_attr_name: xml_attr_value for xml_attr_name, xml_attr_value in child.attrib.items()}
                    # The from_dict method is responsible for converting string values to appropriate types
                    kwargs[python_name] = type_info.from_dict(xml_attr_dict)

                # Branch 2: Simple text element → str with type conversion
                # <Element>value</Element>
                elif child.text and child.text.strip() and len(child) == 0:
                    xml_text_value = child.text.strip()
                    # Use type_info for conversion via _convert_value
                    kwargs[python_name] = self._convert_value(xml_text_value, type_info)

                # Branch 3: Complex element with children → use deserializer
                # <Element><SubElement1 /><SubElement2 /></Element>
                else:
                    deserializer = self.DESERIALIZERS.get(child_name, None)
                    if not deserializer:
                        raise BTLxParsingError("No deserializer found for type: {}".format(child_name), processing_type=processing_name)
                    kwargs[python_name] = deserializer(child)

            elif child_name == "Processings":
                subprocessings.extend(self._parse_processing(nested) for nested in child)

            elif child_name in self._processing_types:
                subprocessings.append(self._parse_processing(child))

        # Create processing instance
        try:
            processing = processing_class(**kwargs)
        except Exception as e:
            raise BTLxParsingError("Failed to instantiate {}: {}".format(processing_name, e), processing_type=processing_name)

        for subprocessing in subprocessings:
            processing.add_subprocessing(subprocessing)
        return processing

    def _parse_insertion_patterns(self, element):
        """Parse the InsertionPatterns XML element of a PatternContour."""
        patterns = []
        for pattern_elem in element:
            position_elem = pattern_elem.find("{*}Position")
            processing_elem = next(child for child in pattern_elem if child is not position_elem)
            patterns.append(InsertionPattern(self._parse_processing(processing_elem), frame_from_xml(position_elem)))
        return InsertionPatterns(patterns)
