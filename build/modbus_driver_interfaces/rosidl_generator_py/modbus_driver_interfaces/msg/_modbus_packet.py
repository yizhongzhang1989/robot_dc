# generated from rosidl_generator_py/resource/_idl.py.em
# with input from modbus_driver_interfaces:msg/ModbusPacket.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'values'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ModbusPacket(type):
    """Metaclass of message 'ModbusPacket'."""

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
                'modbus_driver_interfaces.msg.ModbusPacket')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__modbus_packet
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__modbus_packet
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__modbus_packet
            cls._TYPE_SUPPORT = module.type_support_msg__msg__modbus_packet
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__modbus_packet

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ModbusPacket(metaclass=Metaclass_ModbusPacket):
    """Message class 'ModbusPacket'."""

    __slots__ = [
        '_function_code',
        '_slave_id',
        '_address',
        '_count',
        '_values',
    ]

    _fields_and_field_types = {
        'function_code': 'uint8',
        'slave_id': 'uint8',
        'address': 'uint16',
        'count': 'uint16',
        'values': 'sequence<uint16>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.function_code = kwargs.get('function_code', int())
        self.slave_id = kwargs.get('slave_id', int())
        self.address = kwargs.get('address', int())
        self.count = kwargs.get('count', int())
        self.values = array.array('H', kwargs.get('values', []))

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
        if self.function_code != other.function_code:
            return False
        if self.slave_id != other.slave_id:
            return False
        if self.address != other.address:
            return False
        if self.count != other.count:
            return False
        if self.values != other.values:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def function_code(self):
        """Message field 'function_code'."""
        return self._function_code

    @function_code.setter
    def function_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'function_code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'function_code' field must be an unsigned integer in [0, 255]"
        self._function_code = value

    @builtins.property
    def slave_id(self):
        """Message field 'slave_id'."""
        return self._slave_id

    @slave_id.setter
    def slave_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'slave_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'slave_id' field must be an unsigned integer in [0, 255]"
        self._slave_id = value

    @builtins.property
    def address(self):
        """Message field 'address'."""
        return self._address

    @address.setter
    def address(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'address' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'address' field must be an unsigned integer in [0, 65535]"
        self._address = value

    @builtins.property
    def count(self):
        """Message field 'count'."""
        return self._count

    @count.setter
    def count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'count' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'count' field must be an unsigned integer in [0, 65535]"
        self._count = value

    @builtins.property
    def values(self):
        """Message field 'values'."""
        return self._values

    @values.setter
    def values(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'values' array.array() must have the type code of 'H'"
            self._values = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'values' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._values = array.array('H', value)
