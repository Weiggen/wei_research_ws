// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorArray = require('./SensorArray.js');
let WeightArray = require('./WeightArray.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ExchangeData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.position = null;
      this.role = null;
      this.weights = null;
      this.sensor_scores = null;
      this.operation_range = null;
      this.approx_param = null;
      this.smoke_variance = null;
      this.camera_range = null;
      this.angle_of_view = null;
      this.camera_variance = null;
      this.velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('role')) {
        this.role = initObj.role
      }
      else {
        this.role = new SensorArray();
      }
      if (initObj.hasOwnProperty('weights')) {
        this.weights = initObj.weights
      }
      else {
        this.weights = new WeightArray();
      }
      if (initObj.hasOwnProperty('sensor_scores')) {
        this.sensor_scores = initObj.sensor_scores
      }
      else {
        this.sensor_scores = new WeightArray();
      }
      if (initObj.hasOwnProperty('operation_range')) {
        this.operation_range = initObj.operation_range
      }
      else {
        this.operation_range = 0.0;
      }
      if (initObj.hasOwnProperty('approx_param')) {
        this.approx_param = initObj.approx_param
      }
      else {
        this.approx_param = 0.0;
      }
      if (initObj.hasOwnProperty('smoke_variance')) {
        this.smoke_variance = initObj.smoke_variance
      }
      else {
        this.smoke_variance = 0.0;
      }
      if (initObj.hasOwnProperty('camera_range')) {
        this.camera_range = initObj.camera_range
      }
      else {
        this.camera_range = 0.0;
      }
      if (initObj.hasOwnProperty('angle_of_view')) {
        this.angle_of_view = initObj.angle_of_view
      }
      else {
        this.angle_of_view = 0.0;
      }
      if (initObj.hasOwnProperty('camera_variance')) {
        this.camera_variance = initObj.camera_variance
      }
      else {
        this.camera_variance = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExchangeData
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [role]
    bufferOffset = SensorArray.serialize(obj.role, buffer, bufferOffset);
    // Serialize message field [weights]
    bufferOffset = WeightArray.serialize(obj.weights, buffer, bufferOffset);
    // Serialize message field [sensor_scores]
    bufferOffset = WeightArray.serialize(obj.sensor_scores, buffer, bufferOffset);
    // Serialize message field [operation_range]
    bufferOffset = _serializer.float64(obj.operation_range, buffer, bufferOffset);
    // Serialize message field [approx_param]
    bufferOffset = _serializer.float64(obj.approx_param, buffer, bufferOffset);
    // Serialize message field [smoke_variance]
    bufferOffset = _serializer.float64(obj.smoke_variance, buffer, bufferOffset);
    // Serialize message field [camera_range]
    bufferOffset = _serializer.float64(obj.camera_range, buffer, bufferOffset);
    // Serialize message field [angle_of_view]
    bufferOffset = _serializer.float64(obj.angle_of_view, buffer, bufferOffset);
    // Serialize message field [camera_variance]
    bufferOffset = _serializer.float64(obj.camera_variance, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExchangeData
    let len;
    let data = new ExchangeData(null);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [role]
    data.role = SensorArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [weights]
    data.weights = WeightArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [sensor_scores]
    data.sensor_scores = WeightArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [operation_range]
    data.operation_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [approx_param]
    data.approx_param = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [smoke_variance]
    data.smoke_variance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [camera_range]
    data.camera_range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle_of_view]
    data.angle_of_view = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [camera_variance]
    data.camera_variance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SensorArray.getMessageSize(object.role);
    length += WeightArray.getMessageSize(object.weights);
    length += WeightArray.getMessageSize(object.sensor_scores);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/ExchangeData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2f42667d1e0a184c7cc115f894849626';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    float64 score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExchangeData(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.role !== undefined) {
      resolved.role = SensorArray.Resolve(msg.role)
    }
    else {
      resolved.role = new SensorArray()
    }

    if (msg.weights !== undefined) {
      resolved.weights = WeightArray.Resolve(msg.weights)
    }
    else {
      resolved.weights = new WeightArray()
    }

    if (msg.sensor_scores !== undefined) {
      resolved.sensor_scores = WeightArray.Resolve(msg.sensor_scores)
    }
    else {
      resolved.sensor_scores = new WeightArray()
    }

    if (msg.operation_range !== undefined) {
      resolved.operation_range = msg.operation_range;
    }
    else {
      resolved.operation_range = 0.0
    }

    if (msg.approx_param !== undefined) {
      resolved.approx_param = msg.approx_param;
    }
    else {
      resolved.approx_param = 0.0
    }

    if (msg.smoke_variance !== undefined) {
      resolved.smoke_variance = msg.smoke_variance;
    }
    else {
      resolved.smoke_variance = 0.0
    }

    if (msg.camera_range !== undefined) {
      resolved.camera_range = msg.camera_range;
    }
    else {
      resolved.camera_range = 0.0
    }

    if (msg.angle_of_view !== undefined) {
      resolved.angle_of_view = msg.angle_of_view;
    }
    else {
      resolved.angle_of_view = 0.0
    }

    if (msg.camera_variance !== undefined) {
      resolved.camera_variance = msg.camera_variance;
    }
    else {
      resolved.camera_variance = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Point.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = ExchangeData;
