# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: get_camera_origin.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import geometry_msgs_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='get_camera_origin.proto',
  package='graspit_rpcz',
  serialized_pb='\n\x17get_camera_origin.proto\x12\x0cgraspit_rpcz\x1a\x13geometry_msgs.proto\"\x15\n\x13\x43\x61meraOriginRequest\"3\n\x14\x43\x61meraOriginResponse\x12\x1b\n\x0c\x63\x61meraOrigin\x18\x01 \x02(\x0b\x32\x05.Pose2c\n\x13\x43\x61meraOriginService\x12L\n\x03run\x12!.graspit_rpcz.CameraOriginRequest\x1a\".graspit_rpcz.CameraOriginResponse')




_CAMERAORIGINREQUEST = _descriptor.Descriptor(
  name='CameraOriginRequest',
  full_name='graspit_rpcz.CameraOriginRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=62,
  serialized_end=83,
)


_CAMERAORIGINRESPONSE = _descriptor.Descriptor(
  name='CameraOriginResponse',
  full_name='graspit_rpcz.CameraOriginResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='cameraOrigin', full_name='graspit_rpcz.CameraOriginResponse.cameraOrigin', index=0,
      number=1, type=11, cpp_type=10, label=2,
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
  serialized_start=85,
  serialized_end=136,
)

_CAMERAORIGINRESPONSE.fields_by_name['cameraOrigin'].message_type = geometry_msgs_pb2._POSE
DESCRIPTOR.message_types_by_name['CameraOriginRequest'] = _CAMERAORIGINREQUEST
DESCRIPTOR.message_types_by_name['CameraOriginResponse'] = _CAMERAORIGINRESPONSE

class CameraOriginRequest(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _CAMERAORIGINREQUEST

  # @@protoc_insertion_point(class_scope:graspit_rpcz.CameraOriginRequest)

class CameraOriginResponse(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _CAMERAORIGINRESPONSE

  # @@protoc_insertion_point(class_scope:graspit_rpcz.CameraOriginResponse)


# @@protoc_insertion_point(module_scope)