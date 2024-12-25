// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ExchangeData = require('./ExchangeData.js');

//-----------------------------------------------------------

class ExchangeDataArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExchangeDataArray
    // Serialize message field [data]
    // Serialize the length for message field [data]
    bufferOffset = _serializer.uint32(obj.data.length, buffer, bufferOffset);
    obj.data.forEach((val) => {
      bufferOffset = ExchangeData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExchangeDataArray
    let len;
    let data = new ExchangeDataArray(null);
    // Deserialize message field [data]
    // Deserialize array length for message field [data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.data[i] = ExchangeData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.data.forEach((val) => {
      length += ExchangeData.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/ExchangeDataArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18f160db55a6039398f060d660965da4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ExchangeData[] data
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
    float64 score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExchangeDataArray(null);
    if (msg.data !== undefined) {
      resolved.data = new Array(msg.data.length);
      for (let i = 0; i < resolved.data.length; ++i) {
        resolved.data[i] = ExchangeData.Resolve(msg.data[i]);
      }
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = ExchangeDataArray;
