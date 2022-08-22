# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from seed_robotics/SyncSetPosition.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SyncSetPosition(genpy.Message):
  _md5sum = "c9220e34b4c74855ca69205046aec62f"
  _type = "seed_robotics/SyncSetPosition"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 id1
uint8 id2
int32 position1
int32 position2
"""
  __slots__ = ['id1','id2','position1','position2']
  _slot_types = ['uint8','uint8','int32','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id1,id2,position1,position2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SyncSetPosition, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id1 is None:
        self.id1 = 0
      if self.id2 is None:
        self.id2 = 0
      if self.position1 is None:
        self.position1 = 0
      if self.position2 is None:
        self.position2 = 0
    else:
      self.id1 = 0
      self.id2 = 0
      self.position1 = 0
      self.position2 = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2B2i().pack(_x.id1, _x.id2, _x.position1, _x.position2))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.id1, _x.id2, _x.position1, _x.position2,) = _get_struct_2B2i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2B2i().pack(_x.id1, _x.id2, _x.position1, _x.position2))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.id1, _x.id2, _x.position1, _x.position2,) = _get_struct_2B2i().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B2i = None
def _get_struct_2B2i():
    global _struct_2B2i
    if _struct_2B2i is None:
        _struct_2B2i = struct.Struct("<2B2i")
    return _struct_2B2i
