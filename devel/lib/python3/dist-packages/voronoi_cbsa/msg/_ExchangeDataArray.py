# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from voronoi_cbsa/ExchangeDataArray.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import voronoi_cbsa.msg

class ExchangeDataArray(genpy.Message):
  _md5sum = "18f160db55a6039398f060d660965da4"
  _type = "voronoi_cbsa/ExchangeDataArray"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """ExchangeData[] data
================================================================================
MSG: voronoi_cbsa/ExchangeData
int64               id
geometry_msgs/Point position
SensorArray         role
WeightArray         weights
WeightArray         sensor_scores
float64             operation_range
float64             approx_param
float64             smoke_variance
float64             camera_range
float64             angle_of_view
float64             camera_variance
geometry_msgs/Point velocity

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
float64 score
================================================================================
MSG: voronoi_cbsa/WeightArray
Weight[] weights
================================================================================
MSG: voronoi_cbsa/Weight
string  type
int16   event_id
float64 score"""
  __slots__ = ['data']
  _slot_types = ['voronoi_cbsa/ExchangeData[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ExchangeDataArray, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.data is None:
        self.data = []
    else:
      self.data = []

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
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _x = val1.id
        buff.write(_get_struct_q().pack(_x))
        _v1 = val1.position
        _x = _v1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v2 = val1.role
        length = len(_v2.sensors)
        buff.write(_struct_I.pack(length))
        for val3 in _v2.sensors:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3.score
          buff.write(_get_struct_d().pack(_x))
        _v3 = val1.weights
        length = len(_v3.weights)
        buff.write(_struct_I.pack(length))
        for val3 in _v3.weights:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3
          buff.write(_get_struct_hd().pack(_x.event_id, _x.score))
        _v4 = val1.sensor_scores
        length = len(_v4.weights)
        buff.write(_struct_I.pack(length))
        for val3 in _v4.weights:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3
          buff.write(_get_struct_hd().pack(_x.event_id, _x.score))
        _x = val1
        buff.write(_get_struct_6d().pack(_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance))
        _v5 = val1.velocity
        _x = _v5
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.data is None:
        self.data = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = voronoi_cbsa.msg.ExchangeData()
        start = end
        end += 8
        (val1.id,) = _get_struct_q().unpack(str[start:end])
        _v6 = val1.position
        _x = _v6
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v7 = val1.role
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v7.sensors = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Sensor()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          start = end
          end += 8
          (val3.score,) = _get_struct_d().unpack(str[start:end])
          _v7.sensors.append(val3)
        _v8 = val1.weights
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v8.weights = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Weight()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          _x = val3
          start = end
          end += 10
          (_x.event_id, _x.score,) = _get_struct_hd().unpack(str[start:end])
          _v8.weights.append(val3)
        _v9 = val1.sensor_scores
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v9.weights = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Weight()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          _x = val3
          start = end
          end += 10
          (_x.event_id, _x.score,) = _get_struct_hd().unpack(str[start:end])
          _v9.weights.append(val3)
        _x = val1
        start = end
        end += 48
        (_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance,) = _get_struct_6d().unpack(str[start:end])
        _v10 = val1.velocity
        _x = _v10
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.data.append(val1)
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
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _x = val1.id
        buff.write(_get_struct_q().pack(_x))
        _v11 = val1.position
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v12 = val1.role
        length = len(_v12.sensors)
        buff.write(_struct_I.pack(length))
        for val3 in _v12.sensors:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3.score
          buff.write(_get_struct_d().pack(_x))
        _v13 = val1.weights
        length = len(_v13.weights)
        buff.write(_struct_I.pack(length))
        for val3 in _v13.weights:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3
          buff.write(_get_struct_hd().pack(_x.event_id, _x.score))
        _v14 = val1.sensor_scores
        length = len(_v14.weights)
        buff.write(_struct_I.pack(length))
        for val3 in _v14.weights:
          _x = val3.type
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _x = val3
          buff.write(_get_struct_hd().pack(_x.event_id, _x.score))
        _x = val1
        buff.write(_get_struct_6d().pack(_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance))
        _v15 = val1.velocity
        _x = _v15
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.data is None:
        self.data = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = voronoi_cbsa.msg.ExchangeData()
        start = end
        end += 8
        (val1.id,) = _get_struct_q().unpack(str[start:end])
        _v16 = val1.position
        _x = _v16
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v17 = val1.role
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v17.sensors = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Sensor()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          start = end
          end += 8
          (val3.score,) = _get_struct_d().unpack(str[start:end])
          _v17.sensors.append(val3)
        _v18 = val1.weights
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v18.weights = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Weight()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          _x = val3
          start = end
          end += 10
          (_x.event_id, _x.score,) = _get_struct_hd().unpack(str[start:end])
          _v18.weights.append(val3)
        _v19 = val1.sensor_scores
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        _v19.weights = []
        for i in range(0, length):
          val3 = voronoi_cbsa.msg.Weight()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val3.type = str[start:end].decode('utf-8', 'rosmsg')
          else:
            val3.type = str[start:end]
          _x = val3
          start = end
          end += 10
          (_x.event_id, _x.score,) = _get_struct_hd().unpack(str[start:end])
          _v19.weights.append(val3)
        _x = val1
        start = end
        end += 48
        (_x.operation_range, _x.approx_param, _x.smoke_variance, _x.camera_range, _x.angle_of_view, _x.camera_variance,) = _get_struct_6d().unpack(str[start:end])
        _v20 = val1.velocity
        _x = _v20
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
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
_struct_hd = None
def _get_struct_hd():
    global _struct_hd
    if _struct_hd is None:
        _struct_hd = struct.Struct("<hd")
    return _struct_hd
_struct_q = None
def _get_struct_q():
    global _struct_q
    if _struct_q is None:
        _struct_q = struct.Struct("<q")
    return _struct_q
