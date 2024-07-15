# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from voronoi_cbsa/NeighborInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import voronoi_cbsa.msg

class NeighborInfo(genpy.Message):
  _md5sum = "11f2e8ce0512f33d5dfa93d67585eb65"
  _type = "voronoi_cbsa/NeighborInfo"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int16 id
geometry_msgs/Point position
SensorArray role
float64             operation_range
float64             approx_param
float64             smoke_variance
float64             camera_range
float64             angle_of_view
float64             camera_variance
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: voronoi_cbsa/SensorArray
Sensor[] sensors
================================================================================
MSG: voronoi_cbsa/Sensor
string type
float64 score"""
  __slots__ = ['id','position','role','operation_range','approx_param','smoke_variance','camera_range','angle_of_view','camera_variance']
  _slot_types = ['int16','geometry_msgs/Point','voronoi_cbsa/SensorArray','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,position,role,operation_range,approx_param,smoke_variance,camera_range,angle_of_view,camera_variance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NeighborInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.role is None:
        self.role = voronoi_cbsa.msg.SensorArray()
      if self.operation_range is None:
        self.operation_range = 0.
      if self.approx_param is None:
        self.approx_param = 0.
      if self.smoke_variance is None:
        self.smoke_variance = 0.
      if self.camera_range is None:
        self.camera_range = 0.
      if self.angle_of_view is None:
        self.angle_of_view = 0.
      if self.camera_variance is None:
        self.camera_variance = 0.
    else:
      self.id = 0
      self.position = geometry_msgs.msg.Point()
      self.role = voronoi_cbsa.msg.SensorArray()
      self.operation_range = 0.
      self.approx_param = 0.
      self.smoke_variance = 0.
      self.camera_range = 0.
      self.angle_of_view = 0.
      self.camera_variance = 0.

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
      buff.write(_get_struct_h3d().pack(_x.id, _x.position.x, _x.position.y, _x.position.z))
      length = len(self.role.sensors)
      buff.write(_struct_I.pack(length))
      for val1 in self.role.sensors:
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.score
        buff.write(_get_struct_d().pack(_x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance))
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
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.role is None:
        self.role = voronoi_cbsa.msg.SensorArray()
      end = 0
      _x = self
      start = end
      end += 26
      (_x.id, _x.position.x, _x.position.y, _x.position.z,) = _get_struct_h3d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.role.sensors = []
      for i in range(0, length):
        val1 = voronoi_cbsa.msg.Sensor()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.type = str[start:end]
        start = end
        end += 8
        (val1.score,) = _get_struct_d().unpack(str[start:end])
        self.role.sensors.append(val1)
      _x = self
      start = end
      end += 48
      (_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance,) = _get_struct_6d().unpack(str[start:end])
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
      buff.write(_get_struct_h3d().pack(_x.id, _x.position.x, _x.position.y, _x.position.z))
      length = len(self.role.sensors)
      buff.write(_struct_I.pack(length))
      for val1 in self.role.sensors:
        _x = val1.type
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.score
        buff.write(_get_struct_d().pack(_x))
      _x = self
      buff.write(_get_struct_6d().pack(_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance))
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
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.role is None:
        self.role = voronoi_cbsa.msg.SensorArray()
      end = 0
      _x = self
      start = end
      end += 26
      (_x.id, _x.position.x, _x.position.y, _x.position.z,) = _get_struct_h3d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.role.sensors = []
      for i in range(0, length):
        val1 = voronoi_cbsa.msg.Sensor()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.type = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.type = str[start:end]
        start = end
        end += 8
        (val1.score,) = _get_struct_d().unpack(str[start:end])
        self.role.sensors.append(val1)
      _x = self
      start = end
      end += 48
      (_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
_struct_h3d = None
def _get_struct_h3d():
    global _struct_h3d
    if _struct_h3d is None:
        _struct_h3d = struct.Struct("<h3d")
    return _struct_h3d
