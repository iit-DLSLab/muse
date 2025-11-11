# generated from rosidl_generator_py/resource/_idl.py.em
# with input from state_estimator_msgs:msg/LegOdometry.idl
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
    import numpy.typing
    from ctypes import Structure

    class PyCapsule(Structure):
        pass  # don't need to define the full structure
    from std_msgs.msg import Header


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'lin_vel_lf'
# Member 'lin_vel_rf'
# Member 'lin_vel_lh'
# Member 'lin_vel_rh'
# Member 'base_velocity'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LegOdometry(rosidl_pycommon.interface_base_classes.MessageTypeSupportMeta):
    """Metaclass of message 'LegOdometry'."""

    _CREATE_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_FROM_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _CONVERT_TO_PY: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _DESTROY_ROS_MESSAGE: typing.ClassVar[typing.Optional[PyCapsule]] = None
    _TYPE_SUPPORT: typing.ClassVar[typing.Optional[PyCapsule]] = None

    class LegOdometryConstants(typing.TypedDict):
        pass

    __constants: LegOdometryConstants = {
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
                'state_estimator_msgs.msg.LegOdometry')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__leg_odometry
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__leg_odometry
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__leg_odometry
            cls._TYPE_SUPPORT = module.type_support_msg__msg__leg_odometry
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__leg_odometry

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


class LegOdometry(rosidl_pycommon.interface_base_classes.BaseMessage, metaclass=Metaclass_LegOdometry):
    """Message class 'LegOdometry'."""

    __slots__ = [
        '_header',
        '_lin_vel_lf',
        '_lin_vel_rf',
        '_lin_vel_lh',
        '_lin_vel_rh',
        '_base_velocity',
        '_check_fields',
    ]

    _fields_and_field_types: dict[str, str] = {
        'header': 'std_msgs/Header',
        'lin_vel_lf': 'double[3]',
        'lin_vel_rf': 'double[3]',
        'lin_vel_lh': 'double[3]',
        'lin_vel_rh': 'double[3]',
        'base_velocity': 'double[3]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES: tuple[rosidl_parser.definition.AbstractType, ...] = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
    )

    def __init__(self, *,
                 header: typing.Optional[Header] = None,  # noqa: E501
                 lin_vel_lf: typing.Optional[typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 lin_vel_rf: typing.Optional[typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 lin_vel_lh: typing.Optional[typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 lin_vel_rh: typing.Optional[typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 base_velocity: typing.Optional[typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]] = None,  # noqa: E501
                 check_fields: typing.Optional[bool] = None) -> None:
        if check_fields is not None:
            self._check_fields = check_fields
        else:
            self._check_fields = ros_python_check_fields == '1'
        from std_msgs.msg import Header
        self.header = header if header is not None else Header()
        if lin_vel_lf is None:
            self.lin_vel_lf = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.lin_vel_lf = lin_vel_lf
        if lin_vel_rf is None:
            self.lin_vel_rf = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.lin_vel_rf = lin_vel_rf
        if lin_vel_lh is None:
            self.lin_vel_lh = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.lin_vel_lh = lin_vel_lh
        if lin_vel_rh is None:
            self.lin_vel_rh = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.lin_vel_rh = lin_vel_rh
        if base_velocity is None:
            self.base_velocity = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.base_velocity = base_velocity

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
        if not isinstance(other, LegOdometry):
            return False
        if self.header != other.header:
            return False
        if any(self.lin_vel_lf != other.lin_vel_lf):
            return False
        if any(self.lin_vel_rf != other.lin_vel_rf):
            return False
        if any(self.lin_vel_lh != other.lin_vel_lh):
            return False
        if any(self.lin_vel_rh != other.lin_vel_rh):
            return False
        if any(self.base_velocity != other.base_velocity):
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
    def lin_vel_lf(self) -> typing.Annotated[typing.Any, numpy.typing.NDArray[numpy.float64]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'lin_vel_lf'."""
        return self._lin_vel_lf

    @lin_vel_lf.setter
    def lin_vel_lf(self, value: typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'lin_vel_lf' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'lin_vel_lf' numpy.ndarray() must have a size of 3"
                self._lin_vel_lf = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'lin_vel_lf' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._lin_vel_lf = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def lin_vel_rf(self) -> typing.Annotated[typing.Any, numpy.typing.NDArray[numpy.float64]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'lin_vel_rf'."""
        return self._lin_vel_rf

    @lin_vel_rf.setter
    def lin_vel_rf(self, value: typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'lin_vel_rf' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'lin_vel_rf' numpy.ndarray() must have a size of 3"
                self._lin_vel_rf = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'lin_vel_rf' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._lin_vel_rf = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def lin_vel_lh(self) -> typing.Annotated[typing.Any, numpy.typing.NDArray[numpy.float64]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'lin_vel_lh'."""
        return self._lin_vel_lh

    @lin_vel_lh.setter
    def lin_vel_lh(self, value: typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'lin_vel_lh' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'lin_vel_lh' numpy.ndarray() must have a size of 3"
                self._lin_vel_lh = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'lin_vel_lh' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._lin_vel_lh = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def lin_vel_rh(self) -> typing.Annotated[typing.Any, numpy.typing.NDArray[numpy.float64]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'lin_vel_rh'."""
        return self._lin_vel_rh

    @lin_vel_rh.setter
    def lin_vel_rh(self, value: typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'lin_vel_rh' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'lin_vel_rh' numpy.ndarray() must have a size of 3"
                self._lin_vel_rh = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'lin_vel_rh' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._lin_vel_rh = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def base_velocity(self) -> typing.Annotated[typing.Any, numpy.typing.NDArray[numpy.float64]]:   # typing.Annotated can be remove after mypy 1.16+ see mypy#3004
        """Message field 'base_velocity'."""
        return self._base_velocity

    @base_velocity.setter
    def base_velocity(self, value: typing.Union[numpy.typing.NDArray[numpy.float64], collections.abc.Sequence[float], collections.abc.Set[float], collections.UserList[float]]) -> None:
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float64, \
                    "The 'base_velocity' numpy.ndarray() must have the dtype of 'numpy.float64'"
                assert value.size == 3, \
                    "The 'base_velocity' numpy.ndarray() must have a size of 3"
                self._base_velocity = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'base_velocity' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._base_velocity = numpy.array(value, dtype=numpy.float64)
