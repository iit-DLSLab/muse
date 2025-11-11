# generated from rosidl_generator_py/resource/_idl.py.em
# with input from state_estimator_msgs:msg/ContactDetection.idl
# generated code does not contain a copyright notice

from __future__ import annotations

import collections.abc
from os import getenv
import typing

import rosidl_pycommon.interface_base_classes

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


if typing.TYPE_CHECKING:
    from ctypes import Structure

    class PyCapsule(Structure):
        pass  # don't need to define the full structure
    from std_msgs.msg import Header


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ContactDetection(rosidl_pycommon.interface_base_classes.MessageTypeSupportMeta):
    """Metaclass of message 'ContactDetection'."""

    _CREATE_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_FROM_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_TO_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _DESTROY_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _TYPE_SUPPORT: typing.ClassVar[typing.Optional[PyCapsule]] = None

    class ContactDetectionConstants(typing.TypedDict):
        pass

    __constants: ContactDetectionConstants = {
    }

    @classmethod
    def __import_type_support__(cls) -> None:
        try:
            from rosidl_generator_py import import_type_support  # type: ignore[attr-defined]
            module = import_type_support('state_estimator_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'state_estimator_msgs.msg.ContactDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__contact_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__contact_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__contact_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__contact_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__contact_detection

            from std_msgs.msg import Header
            if Header._TYPE_SUPPORT is None:
                Header.__import_type_support__()

    @classmethod
    def __prepare__(metacls, name: str, bases: tuple[type[typing.Any], ...], /, **kwds: typing.Any) -> collections.abc.MutableMapping[str, object]:
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ContactDetection(rosidl_pycommon.interface_base_classes.BaseMessage, metaclass=Metaclass_ContactDetection):
    """Message class 'ContactDetection'."""

    __slots__ = [
        '_header',
        '_stance_lf',
        '_stance_rf',
        '_stance_lh',
        '_stance_rh',
        '_check_fields',
    ]

    _fields_and_field_types: dict[str, str] = {
        'header': 'std_msgs/Header',
        'stance_lf': 'boolean',
        'stance_rf': 'boolean',
        'stance_lh': 'boolean',
        'stance_rh': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES: tuple[rosidl_parser.definition.AbstractType, ...] = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, *,
                 header: typing.Optional[Header] = None,  # noqa: E501
                 stance_lf: typing.Optional[bool] = None,  # noqa: E501
                 stance_rf: typing.Optional[bool] = None,  # noqa: E501
                 stance_lh: typing.Optional[bool] = None,  # noqa: E501
                 stance_rh: typing.Optional[bool] = None,  # noqa: E501
                 check_fields: typing.Optional[bool] = None) -> None:
        if check_fields is not None:
            self._check_fields = check_fields
        else:
            self._check_fields = ros_python_check_fields == '1'
        from std_msgs.msg import Header
        self.header = header if header is not None else Header()
        self.stance_lf = stance_lf if stance_lf is not None else bool()
        self.stance_rf = stance_rf if stance_rf is not None else bool()
        self.stance_lh = stance_lh if stance_lh is not None else bool()
        self.stance_rh = stance_rh if stance_rh is not None else bool()

    def __repr__(self) -> str:
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args: list[str] = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ContactDetection):
            return False
        if self.header != other.header:
            return False
        if self.stance_lf != other.stance_lf:
            return False
        if self.stance_rf != other.stance_rf:
            return False
        if self.stance_lh != other.stance_lh:
            return False
        if self.stance_rh != other.stance_rh:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls) -> dict[str, str]:
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self) -> Header:
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value: Header) -> None:
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def stance_lf(self) -> bool:
        """Message field 'stance_lf'."""
        return self._stance_lf

    @stance_lf.setter
    def stance_lf(self, value: bool) -> None:
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'stance_lf' field must be of type 'bool'"
        self._stance_lf = value

    @builtins.property
    def stance_rf(self) -> bool:
        """Message field 'stance_rf'."""
        return self._stance_rf

    @stance_rf.setter
    def stance_rf(self, value: bool) -> None:
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'stance_rf' field must be of type 'bool'"
        self._stance_rf = value

    @builtins.property
    def stance_lh(self) -> bool:
        """Message field 'stance_lh'."""
        return self._stance_lh

    @stance_lh.setter
    def stance_lh(self, value: bool) -> None:
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'stance_lh' field must be of type 'bool'"
        self._stance_lh = value

    @builtins.property
    def stance_rh(self) -> bool:
        """Message field 'stance_rh'."""
        return self._stance_rh

    @stance_rh.setter
    def stance_rh(self, value: bool) -> None:
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'stance_rh' field must be of type 'bool'"
        self._stance_rh = value
