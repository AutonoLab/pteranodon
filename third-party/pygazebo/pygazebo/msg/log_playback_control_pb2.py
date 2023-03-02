# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: log_playback_control.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import time_pb2 as time__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='log_playback_control.proto',
  package='gazebo.msgs',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x1alog_playback_control.proto\x12\x0bgazebo.msgs\x1a\ntime.proto\"\x8c\x01\n\x12LogPlaybackControl\x12\r\n\x05pause\x18\x01 \x01(\x08\x12\x12\n\nmulti_step\x18\x02 \x01(\x11\x12\x0e\n\x06rewind\x18\x03 \x01(\x08\x12\x0f\n\x07\x66orward\x18\x04 \x01(\x08\x12\x1f\n\x04seek\x18\x05 \x01(\x0b\x32\x11.gazebo.msgs.Time\x12\x11\n\trt_factor\x18\x06 \x01(\x01')
  ,
  dependencies=[time__pb2.DESCRIPTOR,])




_LOGPLAYBACKCONTROL = _descriptor.Descriptor(
  name='LogPlaybackControl',
  full_name='gazebo.msgs.LogPlaybackControl',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pause', full_name='gazebo.msgs.LogPlaybackControl.pause', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='multi_step', full_name='gazebo.msgs.LogPlaybackControl.multi_step', index=1,
      number=2, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rewind', full_name='gazebo.msgs.LogPlaybackControl.rewind', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='forward', full_name='gazebo.msgs.LogPlaybackControl.forward', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='seek', full_name='gazebo.msgs.LogPlaybackControl.seek', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rt_factor', full_name='gazebo.msgs.LogPlaybackControl.rt_factor', index=5,
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
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=56,
  serialized_end=196,
)

_LOGPLAYBACKCONTROL.fields_by_name['seek'].message_type = time__pb2._TIME
DESCRIPTOR.message_types_by_name['LogPlaybackControl'] = _LOGPLAYBACKCONTROL
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LogPlaybackControl = _reflection.GeneratedProtocolMessageType('LogPlaybackControl', (_message.Message,), dict(
  DESCRIPTOR = _LOGPLAYBACKCONTROL,
  __module__ = 'log_playback_control_pb2'
  # @@protoc_insertion_point(class_scope:gazebo.msgs.LogPlaybackControl)
  ))
_sym_db.RegisterMessage(LogPlaybackControl)


# @@protoc_insertion_point(module_scope)
