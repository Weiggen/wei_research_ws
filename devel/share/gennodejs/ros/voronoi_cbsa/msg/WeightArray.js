// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Weight = require('./Weight.js');

//-----------------------------------------------------------

class WeightArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.weights = null;
    }
    else {
      if (initObj.hasOwnProperty('weights')) {
        this.weights = initObj.weights
      }
      else {
        this.weights = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WeightArray
    // Serialize message field [weights]
    // Serialize the length for message field [weights]
    bufferOffset = _serializer.uint32(obj.weights.length, buffer, bufferOffset);
    obj.weights.forEach((val) => {
      bufferOffset = Weight.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WeightArray
    let len;
    let data = new WeightArray(null);
    // Deserialize message field [weights]
    // Deserialize array length for message field [weights]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.weights = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.weights[i] = Weight.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.weights.forEach((val) => {
      length += Weight.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/WeightArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a52d516a208ee351d816a3ad44a096ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new WeightArray(null);
    if (msg.weights !== undefined) {
      resolved.weights = new Array(msg.weights.length);
      for (let i = 0; i < resolved.weights.length; ++i) {
        resolved.weights[i] = Weight.Resolve(msg.weights[i]);
      }
    }
    else {
      resolved.weights = []
    }

    return resolved;
    }
};

module.exports = WeightArray;
