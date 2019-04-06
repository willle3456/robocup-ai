# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: grSim_Packet.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database

# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()

from proto import grSim_Commands_pb2, grSim_Replacement_pb2

DESCRIPTOR = _descriptor.FileDescriptor(
  name='grSim_Packet.proto',
  package='',
  serialized_pb=_b('\n\x12grSim_Packet.proto\x1a\x14grSim_Commands.proto\x1a\x17grSim_Replacement.proto\"Z\n\x0cgrSim_Packet\x12!\n\x08\x63ommands\x18\x01 \x01(\x0b\x32\x0f.grSim_Commands\x12\'\n\x0breplacement\x18\x02 \x01(\x0b\x32\x12.grSim_Replacement')
  ,
  dependencies=[grSim_Commands_pb2.DESCRIPTOR, grSim_Replacement_pb2.DESCRIPTOR, ])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_GRSIM_PACKET = _descriptor.Descriptor(
  name='grSim_Packet',
  full_name='grSim_Packet',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='commands', full_name='grSim_Packet.commands', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='replacement', full_name='grSim_Packet.replacement', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=69,
  serialized_end=159,
)

_GRSIM_PACKET.fields_by_name['commands'].message_type = grSim_Commands_pb2._GRSIM_COMMANDS
_GRSIM_PACKET.fields_by_name['replacement'].message_type = grSim_Replacement_pb2._GRSIM_REPLACEMENT
DESCRIPTOR.message_types_by_name['grSim_Packet'] = _GRSIM_PACKET

grSim_Packet = _reflection.GeneratedProtocolMessageType('grSim_Packet', (_message.Message,), dict(
  DESCRIPTOR = _GRSIM_PACKET,
  __module__ = 'grSim_Packet_pb2'
  # @@protoc_insertion_point(class_scope:grSim_Packet)
  ))
_sym_db.RegisterMessage(grSim_Packet)


# @@protoc_insertion_point(module_scope)
