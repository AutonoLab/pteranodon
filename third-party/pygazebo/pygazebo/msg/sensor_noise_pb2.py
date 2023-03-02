# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_noise.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_noise.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x12sensor_noise.proto\x12\x0bgazebo.msgs\"\xcb\x01\n\x0bSensorNoise\x12+\n\x04type\x18\x01 \x02(\x0e\x32\x1d.gazebo.msgs.SensorNoise.Type\x12\x0c\n\x04mean\x18\x02 \x01(\x01\x12\x0e\n\x06stddev\x18\x03 \x01(\x01\x12\x11\n\tbias_mean\x18\x04 \x01(\x01\x12\x13\n\x0b\x62ias_stddev\x18\x05 \x01(\x01\x12\x11\n\tprecision\x18\x06 \x01(\x01\"6\n\x04Type\x12\x08\n\x04NONE\x10\x01\x12\x0c\n\x08GAUSSIAN\x10\x02\x12\x16\n\x12GAUSSIAN_QUANTIZED\x10\x03')
)



_SENSORNOISE_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='gazebo.msgs.SensorNoise.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GAUSSIAN', index=1, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GAUSSIAN_QUANTIZED', index=2, number=3,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=185,
  serialized_end=239,
)
_sym_db.RegisterEnumDescriptor(_SENSORNOISE_TYPE)


_SENSORNOISE = _descriptor.Descriptor(
  name='SensorNoise',
  full_name='gazebo.msgs.SensorNoise',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='gazebo.msgs.SensorNoise.type', index=0,
      number=1, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mean', full_name='gazebo.msgs.SensorNoise.mean', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stddev', full_name='gazebo.msgs.SensorNoise.stddev', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bias_mean', full_name='gazebo.msgs.SensorNoise.bias_mean', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bias_stddev', full_name='gazebo.msgs.SensorNoise.bias_stddev', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='precision', full_name='gazebo.msgs.SensorNoise.precision', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SENSORNOISE_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=36,
  serialized_end=239,
)

_SENSORNOISE.fields_by_name['type'].enum_type = _SENSORNOISE_TYPE
_SENSORNOISE_TYPE.containing_type = _SENSORNOISE
DESCRIPTOR.message_types_by_name['SensorNoise'] = _SENSORNOISE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SensorNoise = _reflection.GeneratedProtocolMessageType('SensorNoise', (_message.Message,), dict(
  DESCRIPTOR = _SENSORNOISE,
  __module__ = 'sensor_noise_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.SensorNoise)
  ))
_sym_db.RegisterMessage(SensorNoise)


# @@protoc_insertion_point(module_scope)
