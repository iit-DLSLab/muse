# generated from rosidl_generator_py/resource/_idl.py.em
# with input from state_estimator_msgs:msg/JointStateWithAcceleration.idl
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
    import collections
    from ctypes import Structure

    class PyCapsule(Structure):
        pass  # don't need to define the full structure
    from std_msgs.msg import Header


# Import statements for member types

# Member 'position'
# Member 'velocity'
# Member 'acceleration'
# Member 'effort'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_JointStateWithAcceleration(rosidl_pycommon.interface_base_classes.MessageTypeSupportMeta):
    """Metaclass of message 'JointStateWithAcceleration'."""

    _CREATE_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_FROM_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_TO_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _DESTROY_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _TYPE_SUPPORT: typing.ClassVar[typing.Optional[PyCapsule]] = None

    class JointStateWithAccelerationConstants(typing.TypedDict):
        pass

    __constants: JointStateWithAccelerationConstants = {
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
                'state_estimator_msgs.msg.JointStateWithAcceleration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joint_state_with_acceleration
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joint_state_with_acceleration
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joint_state_with_acceleration
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joint_state_with_acceleration
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joint_state_with_acceleration

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


class JointStateWithAcceleration(rosidl_pycommon.interface_base_classes.BaseMessage, metaclass=Metaclass_JointStateWithAcceleration):
    """Message class 'JointStateWithAcceleration'."""

    __slots__ = [
        '_header',
        '_name',
        '_position',
        '_velocity',
        '_acceleration',
        '_effort',
        '_check_fields',
    ]

    _fields_and_field_types: dict[str, str] = {
        'header': 'std_msgs/Header',
        'name': 'sequence<string>',
        'position': 'sequence<double>',
        'velocity': 'sequence<double>',
        'acceleration': 'sequence<double>',
        'effort': 'sequence<double>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES: tuple[rosidl_parser.definition.AbstractType, ...] = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
    )

    def __init__(self, *,
                 header: typing.Optional[Header] = None,  # noqa: E501
                 name: typing.Optional[typing.Union[collections.abc.Sequence[str], collections.abc.Set[str], collections.UserList[str]]] = None,  # noqa: E501
                 position: typing.Optional[typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 velocity: typing.Optional[typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 acceleration: typing.Optional[typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 effort: typing.Optional[typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 check_fields: typing.Optional[bool] = None) -> None:
        if check_fields is not None:
            self._check_fields = check_fields
        else:
            self._check_fields = ros_python_check_fields == '1'
        from std_msgs.msg import Header
        self.header = header if header is not None else Header()
        self.name = name if name is not None else []
        self.position = position if position is not None else array.array('d', [])
        self.velocity = velocity if velocity is not None else array.array('d', [])
        self.acceleration = acceleration if acceleration is not None else array.array('d', [])
        self.effort = effort if effort is not None else array.array('d', [])

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
        if not isinstance(other, JointStateWithAcceleration):
            return False
        if self.header != other.header:
            return False
        if self.name != other.name:
            return False
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.acceleration != other.acceleration:
            return False
        if self.effort != other.effort:
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
    def name(self) -> typing.Union[collections.abc.Sequence[str], collections.abc.Set[str], collections.UserList[str]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value: typing.Union[collections.abc.Sequence[str], collections.abc.Set[str], collections.UserList[str]]) -> None:
        if self._check_fields:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'name' field must be a set or sequence and each value of type 'str'"
        self._name = value

    @builtins.property
    def position(self) -> typing.Annotated[typing.Any, array.array[float]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value: typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'position' array.array() must have the type code of 'd'"
                self._position = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'position' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        # type ignore below fixed in mypy 1.17+ see mypy#19421
        self._position = array.array('d', value)  # type: ignore[assignment]

    @builtins.property
    def velocity(self) -> typing.Annotated[typing.Any, array.array[float]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value: typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'velocity' array.array() must have the type code of 'd'"
                self._velocity = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'velocity' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        # type ignore below fixed in mypy 1.17+ see mypy#19421
        self._velocity = array.array('d', value)  # type: ignore[assignment]

    @builtins.property
    def acceleration(self) -> typing.Annotated[typing.Any, array.array[float]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value: typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'acceleration' array.array() must have the type code of 'd'"
                self._acceleration = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'acceleration' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        # type ignore below fixed in mypy 1.17+ see mypy#19421
        self._acceleration = array.array('d', value)  # type: ignore[assignment]

    @builtins.property
    def effort(self) -> typing.Annotated[typing.Any, array.array[float]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'effort'."""
        return self._effort

    @effort.setter
    def effort(self, value: typing.Union[array.array[float], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'd', \
                    "The 'effort' array.array() must have the type code of 'd'"
                self._effort = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'effort' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        # type ignore below fixed in mypy 1.17+ see mypy#19421
        self._effort = array.array('d', value)  # type: ignore[assignment]
