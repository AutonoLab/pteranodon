# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: logical_camera_sensor.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='logical_camera_sensor.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x1blogical_camera_sensor.proto\x12\x0bgazebo.msgs\"h\n\x13LogicalCameraSensor\x12\x11\n\tnear_clip\x18\x01 \x02(\x01\x12\x10\n\x08\x66\x61r_clip\x18\x02 \x02(\x01\x12\x16\n\x0ehorizontal_fov\x18\x03 \x02(\x01\x12\x14\n\x0c\x61spect_ratio\x18\x04 \x02(\x01')
)




_LOGICALCAMERASENSOR = _descriptor.Descriptor(
  name='LogicalCameraSensor',
  full_name='gazebo.msgs.LogicalCameraSensor',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='near_clip', full_name='gazebo.msgs.LogicalCameraSensor.near_clip', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='far_clip', full_name='gazebo.msgs.LogicalCameraSensor.far_clip', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='horizontal_fov', full_name='gazebo.msgs.LogicalCameraSensor.horizontal_fov', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='aspect_ratio', full_name='gazebo.msgs.LogicalCameraSensor.aspect_ratio', index=3,
      number=4, type=1, cpp_type=5, label=2,
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
  serialized_start=44,
  serialized_end=148,
)

DESCRIPTOR.message_types_by_name['LogicalCameraSensor'] = _LOGICALCAMERASENSOR
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LogicalCameraSensor = _reflection.GeneratedProtocolMessageType('LogicalCameraSensor', (_message.Message,), dict(
  DESCRIPTOR = _LOGICALCAMERASENSOR,
  __module__ = 'logical_camera_sensor_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.LogicalCameraSensor)
  ))
_sym_db.RegisterMessage(LogicalCameraSensor)


# @@protoc_insertion_point(module_scope)
