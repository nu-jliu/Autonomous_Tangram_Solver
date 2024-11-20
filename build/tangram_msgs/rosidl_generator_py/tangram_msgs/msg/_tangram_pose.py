# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tangram_msgs:msg/TangramPose.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TangramPose(type):
    """Metaclass of message 'TangramPose'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'BIG_TRIANGLE': 1,
        'MEDIUM_TRIANGLE': 2,
        'SMALL_TRIANGLE': 3,
        'PARALELOGRAM': 4,
        'SQUARE': 5,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tangram_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tangram_msgs.msg.TangramPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tangram_pose
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tangram_pose
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tangram_pose
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tangram_pose
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tangram_pose

            from geometry_msgs.msg import Pose2D
            if Pose2D.__class__._TYPE_SUPPORT is None:
                Pose2D.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'BIG_TRIANGLE': cls.__constants['BIG_TRIANGLE'],
            'MEDIUM_TRIANGLE': cls.__constants['MEDIUM_TRIANGLE'],
            'SMALL_TRIANGLE': cls.__constants['SMALL_TRIANGLE'],
            'PARALELOGRAM': cls.__constants['PARALELOGRAM'],
            'SQUARE': cls.__constants['SQUARE'],
        }

    @property
    def BIG_TRIANGLE(self):
        """Message constant 'BIG_TRIANGLE'."""
        return Metaclass_TangramPose.__constants['BIG_TRIANGLE']

    @property
    def MEDIUM_TRIANGLE(self):
        """Message constant 'MEDIUM_TRIANGLE'."""
        return Metaclass_TangramPose.__constants['MEDIUM_TRIANGLE']

    @property
    def SMALL_TRIANGLE(self):
        """Message constant 'SMALL_TRIANGLE'."""
        return Metaclass_TangramPose.__constants['SMALL_TRIANGLE']

    @property
    def PARALELOGRAM(self):
        """Message constant 'PARALELOGRAM'."""
        return Metaclass_TangramPose.__constants['PARALELOGRAM']

    @property
    def SQUARE(self):
        """Message constant 'SQUARE'."""
        return Metaclass_TangramPose.__constants['SQUARE']


class TangramPose(metaclass=Metaclass_TangramPose):
    """
    Message class 'TangramPose'.

    Constants:
      BIG_TRIANGLE
      MEDIUM_TRIANGLE
      SMALL_TRIANGLE
      PARALELOGRAM
      SQUARE
    """

    __slots__ = [
        '_pose',
        '_flip',
        '_type',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/Pose2D',
        'flip': 'boolean',
        'type': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose2D'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose2D
        self.pose = kwargs.get('pose', Pose2D())
        self.flip = kwargs.get('flip', bool())
        self.type = kwargs.get('type', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
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

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.pose != other.pose:
            return False
        if self.flip != other.flip:
            return False
        if self.type != other.type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Pose2D
            assert \
                isinstance(value, Pose2D), \
                "The 'pose' field must be a sub message of type 'Pose2D'"
        self._pose = value

    @builtins.property
    def flip(self):
        """Message field 'flip'."""
        return self._flip

    @flip.setter
    def flip(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'flip' field must be of type 'bool'"
        self._flip = value

    @builtins.property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'type' field must be an integer in [-2147483648, 2147483647]"
        self._type = value
