// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NeighborInfo = require('./NeighborInfo.js');

//-----------------------------------------------------------

class NeighborInfoArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.neighbors = null;
    }
    else {
      if (initObj.hasOwnProperty('neighbors')) {
        this.neighbors = initObj.neighbors
      }
      else {
        this.neighbors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NeighborInfoArray
    // Serialize message field [neighbors]
    // Serialize the length for message field [neighbors]
    bufferOffset = _serializer.uint32(obj.neighbors.length, buffer, bufferOffset);
    obj.neighbors.forEach((val) => {
      bufferOffset = NeighborInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NeighborInfoArray
    let len;
    let data = new NeighborInfoArray(null);
    // Deserialize message field [neighbors]
    // Deserialize array length for message field [neighbors]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.neighbors = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.neighbors[i] = NeighborInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.neighbors.forEach((val) => {
      length += NeighborInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/NeighborInfoArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc1d874a9451abd6cbb97a2b631e4d4f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    NeighborInfo[] neighbors
    ================================================================================
    MSG: voronoi_cbsa/NeighborInfo
    int16 id
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
    float64 score
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NeighborInfoArray(null);
    if (msg.neighbors !== undefined) {
      resolved.neighbors = new Array(msg.neighbors.length);
      for (let i = 0; i < resolved.neighbors.length; ++i) {
        resolved.neighbors[i] = NeighborInfo.Resolve(msg.neighbors[i]);
      }
    }
    else {
      resolved.neighbors = []
    }

    return resolved;
    }
};

module.exports = NeighborInfoArray;
