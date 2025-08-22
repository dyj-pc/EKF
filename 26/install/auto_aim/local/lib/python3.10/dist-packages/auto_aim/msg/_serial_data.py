# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim:msg/SerialData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SerialData(type):
    """Metaclass of message 'SerialData'."""

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
            module = import_type_support('auto_aim')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim.msg.SerialData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__serial_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__serial_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__serial_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__serial_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__serial_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SerialData(metaclass=Metaclass_SerialData):
    """Message class 'SerialData'."""

    __slots__ = [
        '_bullet_velocity',
        '_bullet_angle',
        '_gimbal_yaw',
        '_color',
    ]

    _fields_and_field_types = {
        'bullet_velocity': 'float',
        'bullet_angle': 'float',
        'gimbal_yaw': 'int16',
        'color': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.bullet_velocity = kwargs.get('bullet_velocity', float())
        self.bullet_angle = kwargs.get('bullet_angle', float())
        self.gimbal_yaw = kwargs.get('gimbal_yaw', int())
        self.color = kwargs.get('color', int())

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
        if self.bullet_velocity != other.bullet_velocity:
            return False
        if self.bullet_angle != other.bullet_angle:
            return False
        if self.gimbal_yaw != other.gimbal_yaw:
            return False
        if self.color != other.color:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bullet_velocity(self):
        """Message field 'bullet_velocity'."""
        return self._bullet_velocity

    @bullet_velocity.setter
    def bullet_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bullet_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bullet_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bullet_velocity = value

    @builtins.property
    def bullet_angle(self):
        """Message field 'bullet_angle'."""
        return self._bullet_angle

    @bullet_angle.setter
    def bullet_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bullet_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bullet_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bullet_angle = value

    @builtins.property
    def gimbal_yaw(self):
        """Message field 'gimbal_yaw'."""
        return self._gimbal_yaw

    @gimbal_yaw.setter
    def gimbal_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gimbal_yaw' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'gimbal_yaw' field must be an integer in [-32768, 32767]"
        self._gimbal_yaw = value

    @builtins.property
    def color(self):
        """Message field 'color'."""
        return self._color

    @color.setter
    def color(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'color' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'color' field must be an unsigned integer in [0, 255]"
        self._color = value
