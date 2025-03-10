// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TargetInfo = require('./TargetInfo.js');

//-----------------------------------------------------------

class TargetInfoArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.targets = null;
    }
    else {
      if (initObj.hasOwnProperty('targets')) {
        this.targets = initObj.targets
      }
      else {
        this.targets = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TargetInfoArray
    // Serialize message field [targets]
    // Serialize the length for message field [targets]
    bufferOffset = _serializer.uint32(obj.targets.length, buffer, bufferOffset);
    obj.targets.forEach((val) => {
      bufferOffset = TargetInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TargetInfoArray
    let len;
    let data = new TargetInfoArray(null);
    // Deserialize message field [targets]
    // Deserialize array length for message field [targets]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.targets = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.targets[i] = TargetInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.targets.forEach((val) => {
      length += TargetInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/TargetInfoArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb14593f2221c85378b0d6d6b83f5b15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TargetInfo[] targets
    ================================================================================
    MSG: voronoi_cbsa/TargetInfo
    int64                   id
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
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TargetInfoArray(null);
    if (msg.targets !== undefined) {
      resolved.targets = new Array(msg.targets.length);
      for (let i = 0; i < resolved.targets.length; ++i) {
        resolved.targets[i] = TargetInfo.Resolve(msg.targets[i]);
      }
    }
    else {
      resolved.targets = []
    }

    return resolved;
    }
};

module.exports = TargetInfoArray;
