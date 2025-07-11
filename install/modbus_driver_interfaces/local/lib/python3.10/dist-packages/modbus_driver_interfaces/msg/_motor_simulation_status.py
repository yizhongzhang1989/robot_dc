# generated from rosidl_generator_py/resource/_idl.py.em
# with input from modbus_driver_interfaces:msg/MotorSimulationStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorSimulationStatus(type):
    """Metaclass of message 'MotorSimulationStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('modbus_driver_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'modbus_driver_interfaces.msg.MotorSimulationStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_simulation_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_simulation_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_simulation_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_simulation_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_simulation_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorSimulationStatus(metaclass=Metaclass_MotorSimulationStatus):
    """Message class 'MotorSimulationStatus'."""

    __slots__ = [
        '_motor_id',
        '_current_position',
        '_target_position',
        '_velocity',
        '_motion_mode',
        '_moving',
        '_last_command',
    ]

    _fields_and_field_types = {
        'motor_id': 'int32',
        'current_position': 'float',
        'target_position': 'float',
        'velocity': 'float',
        'motion_mode': 'int32',
        'moving': 'boolean',
        'last_command': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.motor_id = kwargs.get('motor_id', int())
        self.current_position = kwargs.get('current_position', float())
        self.target_position = kwargs.get('target_position', float())
        self.velocity = kwargs.get('velocity', float())
        self.motion_mode = kwargs.get('motion_mode', int())
        self.moving = kwargs.get('moving', bool())
        self.last_command = kwargs.get('last_command', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.motor_id != other.motor_id:
            return False
        if self.current_position != other.current_position:
            return False
        if self.target_position != other.target_position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.motion_mode != other.motion_mode:
            return False
        if self.moving != other.moving:
            return False
        if self.last_command != other.last_command:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def motor_id(self):
        """Message field 'motor_id'."""
        return self._motor_id

    @motor_id.setter
    def motor_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motor_id' field must be an integer in [-2147483648, 2147483647]"
        self._motor_id = value

    @builtins.property
    def current_position(self):
        """Message field 'current_position'."""
        return self._current_position

    @current_position.setter
    def current_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current_position' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current_position' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current_position = value

    @builtins.property
    def target_position(self):
        """Message field 'target_position'."""
        return self._target_position

    @target_position.setter
    def target_position(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'target_position' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'target_position' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._target_position = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._velocity = value

    @builtins.property
    def motion_mode(self):
        """Message field 'motion_mode'."""
        return self._motion_mode

    @motion_mode.setter
    def motion_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motion_mode' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'motion_mode' field must be an integer in [-2147483648, 2147483647]"
        self._motion_mode = value

    @builtins.property
    def moving(self):
        """Message field 'moving'."""
        return self._moving

    @moving.setter
    def moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'moving' field must be of type 'bool'"
        self._moving = value

    @builtins.property
    def last_command(self):
        """Message field 'last_command'."""
        return self._last_command

    @last_command.setter
    def last_command(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'last_command' field must be of type 'str'"
        self._last_command = value
