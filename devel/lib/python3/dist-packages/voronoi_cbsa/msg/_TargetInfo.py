# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from voronoi_cbsa/TargetInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class TargetInfo(genpy.Message):
  _md5sum = "a11d744703fb2fb21b1eb6c816c6cd6f"
  _type = "voronoi_cbsa/TargetInfo"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int64                   id
geometry_msgs/Point     position
float32                 height
float64[]               covariance
float32                 weight
geometry_msgs/Twist     velocity
string[]                required_sensor
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['id','position','height','covariance','weight','velocity','required_sensor']
  _slot_types = ['int64','geometry_msgs/Point','float32','float64[]','float32','geometry_msgs/Twist','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,position,height,covariance,weight,velocity,required_sensor

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TargetInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = 0
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.height is None:
        self.height = 0.
      if self.covariance is None:
        self.covariance = []
      if self.weight is None:
        self.weight = 0.
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      if self.required_sensor is None:
        self.required_sensor = []
    else:
      self.id = 0
      self.position = geometry_msgs.msg.Point()
      self.height = 0.
      self.covariance = []
      self.weight = 0.
      self.velocity = geometry_msgs.msg.Twist()
      self.required_sensor = []

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
      buff.write(_get_struct_q3df().pack(_x.id, _x.position.x, _x.position.y, _x.position.z, _x.height))
      length = len(self.covariance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.covariance))
      _x = self
      buff.write(_get_struct_f6d().pack(_x.weight, _x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z))
      length = len(self.required_sensor)
      buff.write(_struct_I.pack(length))
      for val1 in self.required_sensor:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
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
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 36
      (_x.id, _x.position.x, _x.position.y, _x.position.z, _x.height,) = _get_struct_q3df().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.covariance = s.unpack(str[start:end])
      _x = self
      start = end
      end += 52
      (_x.weight, _x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z,) = _get_struct_f6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.required_sensor = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.required_sensor.append(val1)
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
      buff.write(_get_struct_q3df().pack(_x.id, _x.position.x, _x.position.y, _x.position.z, _x.height))
      length = len(self.covariance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.covariance.tostring())
      _x = self
      buff.write(_get_struct_f6d().pack(_x.weight, _x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z))
      length = len(self.required_sensor)
      buff.write(_struct_I.pack(length))
      for val1 in self.required_sensor:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.Struct('<I%ss'%length).pack(length, val1))
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
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 36
      (_x.id, _x.position.x, _x.position.y, _x.position.z, _x.height,) = _get_struct_q3df().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 52
      (_x.weight, _x.velocity.linear.x, _x.velocity.linear.y, _x.velocity.linear.z, _x.velocity.angular.x, _x.velocity.angular.y, _x.velocity.angular.z,) = _get_struct_f6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.required_sensor = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1 = str[start:end]
        self.required_sensor.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_f6d = None
def _get_struct_f6d():
    global _struct_f6d
    if _struct_f6d is None:
        _struct_f6d = struct.Struct("<f6d")
    return _struct_f6d
_struct_q3df = None
def _get_struct_q3df():
    global _struct_q3df
    if _struct_q3df is None:
        _struct_q3df = struct.Struct("<q3df")
    return _struct_q3df
