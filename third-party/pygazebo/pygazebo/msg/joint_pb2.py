# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: joint.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import axis_pb2 as axis__pb2
from . import pose_pb2 as pose__pb2
from . import sensor_pb2 as sensor__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='joint.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x0bjoint.proto\x12\x0bgazebo.msgs\x1a\naxis.proto\x1a\npose.proto\x1a\x0csensor.proto\"\xe7\x05\n\x05Joint\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\n\n\x02id\x18\x02 \x01(\r\x12\r\n\x05\x61ngle\x18\x03 \x03(\x01\x12%\n\x04type\x18\x04 \x01(\x0e\x32\x17.gazebo.msgs.Joint.Type\x12\x0e\n\x06parent\x18\x05 \x01(\t\x12\x11\n\tparent_id\x18\x06 \x01(\r\x12\r\n\x05\x63hild\x18\x07 \x01(\t\x12\x10\n\x08\x63hild_id\x18\x08 \x01(\r\x12\x1f\n\x04pose\x18\t \x01(\x0b\x32\x11.gazebo.msgs.Pose\x12 \n\x05\x61xis1\x18\n \x01(\x0b\x32\x11.gazebo.msgs.Axis\x12 \n\x05\x61xis2\x18\x0b \x01(\x0b\x32\x11.gazebo.msgs.Axis\x12\x0b\n\x03\x63\x66m\x18\x0c \x01(\x01\x12\x0e\n\x06\x62ounce\x18\r \x01(\x01\x12\x10\n\x08velocity\x18\x0e \x01(\x01\x12\x14\n\x0c\x66udge_factor\x18\x0f \x01(\x01\x12\x11\n\tlimit_cfm\x18\x10 \x01(\x01\x12\x11\n\tlimit_erp\x18\x11 \x01(\x01\x12\x16\n\x0esuspension_cfm\x18\x12 \x01(\x01\x12\x16\n\x0esuspension_erp\x18\x13 \x01(\x01\x12+\n\x07gearbox\x18\x14 \x01(\x0b\x32\x1a.gazebo.msgs.Joint.Gearbox\x12\'\n\x05screw\x18\x15 \x01(\x0b\x32\x18.gazebo.msgs.Joint.Screw\x12#\n\x06sensor\x18\x16 \x03(\x0b\x32\x13.gazebo.msgs.Sensor\x1a@\n\x07Gearbox\x12\x1e\n\x16gearbox_reference_body\x18\x01 \x02(\t\x12\x15\n\rgearbox_ratio\x18\x02 \x02(\x01\x1a\x1d\n\x05Screw\x12\x14\n\x0cthread_pitch\x18\x01 \x02(\x01\"n\n\x04Type\x12\x0c\n\x08REVOLUTE\x10\x01\x12\r\n\tREVOLUTE2\x10\x02\x12\r\n\tPRISMATIC\x10\x03\x12\r\n\tUNIVERSAL\x10\x04\x12\x08\n\x04\x42\x41LL\x10\x05\x12\t\n\x05SCREW\x10\x06\x12\x0b\n\x07GEARBOX\x10\x07\x12\t\n\x05\x46IXED\x10\x08')
  ,
  dependencies=[axis__pb2.DESCRIPTOR,pose__pb2.DESCRIPTOR,sensor__pb2.DESCRIPTOR,])



_JOINT_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='gazebo.msgs.Joint.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='REVOLUTE', index=0, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REVOLUTE2', index=1, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PRISMATIC', index=2, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNIVERSAL', index=3, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BALL', index=4, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SCREW', index=5, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GEARBOX', index=6, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIXED', index=7, number=8,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=700,
  serialized_end=810,
)
_sym_db.RegisterEnumDescriptor(_JOINT_TYPE)


_JOINT_GEARBOX = _descriptor.Descriptor(
  name='Gearbox',
  full_name='gazebo.msgs.Joint.Gearbox',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='gearbox_reference_body', full_name='gazebo.msgs.Joint.Gearbox.gearbox_reference_body', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gearbox_ratio', full_name='gazebo.msgs.Joint.Gearbox.gearbox_ratio', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=603,
  serialized_end=667,
)

_JOINT_SCREW = _descriptor.Descriptor(
  name='Screw',
  full_name='gazebo.msgs.Joint.Screw',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='thread_pitch', full_name='gazebo.msgs.Joint.Screw.thread_pitch', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=669,
  serialized_end=698,
)

_JOINT = _descriptor.Descriptor(
  name='Joint',
  full_name='gazebo.msgs.Joint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='gazebo.msgs.Joint.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='gazebo.msgs.Joint.id', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angle', full_name='gazebo.msgs.Joint.angle', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='type', full_name='gazebo.msgs.Joint.type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parent', full_name='gazebo.msgs.Joint.parent', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='parent_id', full_name='gazebo.msgs.Joint.parent_id', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='child', full_name='gazebo.msgs.Joint.child', index=6,
      number=7, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='child_id', full_name='gazebo.msgs.Joint.child_id', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pose', full_name='gazebo.msgs.Joint.pose', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='axis1', full_name='gazebo.msgs.Joint.axis1', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='axis2', full_name='gazebo.msgs.Joint.axis2', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cfm', full_name='gazebo.msgs.Joint.cfm', index=11,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bounce', full_name='gazebo.msgs.Joint.bounce', index=12,
      number=13, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='velocity', full_name='gazebo.msgs.Joint.velocity', index=13,
      number=14, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='fudge_factor', full_name='gazebo.msgs.Joint.fudge_factor', index=14,
      number=15, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='limit_cfm', full_name='gazebo.msgs.Joint.limit_cfm', index=15,
      number=16, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='limit_erp', full_name='gazebo.msgs.Joint.limit_erp', index=16,
      number=17, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='suspension_cfm', full_name='gazebo.msgs.Joint.suspension_cfm', index=17,
      number=18, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='suspension_erp', full_name='gazebo.msgs.Joint.suspension_erp', index=18,
      number=19, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gearbox', full_name='gazebo.msgs.Joint.gearbox', index=19,
      number=20, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='screw', full_name='gazebo.msgs.Joint.screw', index=20,
      number=21, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sensor', full_name='gazebo.msgs.Joint.sensor', index=21,
      number=22, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_JOINT_GEARBOX, _JOINT_SCREW, ],
  enum_types=[
    _JOINT_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=67,
  serialized_end=810,
)

_JOINT_GEARBOX.containing_type = _JOINT
_JOINT_SCREW.containing_type = _JOINT
_JOINT.fields_by_name['type'].enum_type = _JOINT_TYPE
_JOINT.fields_by_name['pose'].message_type = pose__pb2._POSE
_JOINT.fields_by_name['axis1'].message_type = axis__pb2._AXIS
_JOINT.fields_by_name['axis2'].message_type = axis__pb2._AXIS
_JOINT.fields_by_name['gearbox'].message_type = _JOINT_GEARBOX
_JOINT.fields_by_name['screw'].message_type = _JOINT_SCREW
_JOINT.fields_by_name['sensor'].message_type = sensor__pb2._SENSOR
_JOINT_TYPE.containing_type = _JOINT
DESCRIPTOR.message_types_by_name['Joint'] = _JOINT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Joint = _reflection.GeneratedProtocolMessageType('Joint', (_message.Message,), dict(

  Gearbox = _reflection.GeneratedProtocolMessageType('Gearbox', (_message.Message,), dict(
    DESCRIPTOR = _JOINT_GEARBOX,
    __module__ = 'joint_pb2'
    # @@protoc_insertion_point(class_scope:gazebo.msgs.Joint.Gearbox)
    ))
  ,

  Screw = _reflection.GeneratedProtocolMessageType('Screw', (_message.Message,), dict(
    DESCRIPTOR = _JOINT_SCREW,
    __module__ = 'joint_pb2'
    # @@protoc_insertion_point(class_scope:gazebo.msgs.Joint.Screw)
    ))
  ,
  DESCRIPTOR = _JOINT,
  __module__ = 'joint_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.Joint)
  ))
_sym_db.RegisterMessage(Joint)
_sym_db.RegisterMessage(Joint.Gearbox)
_sym_db.RegisterMessage(Joint.Screw)


# @@protoc_insertion_point(module_scope)
