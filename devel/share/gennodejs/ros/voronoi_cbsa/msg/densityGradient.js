// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class densityGradient {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.gradient_x = null;
      this.gradient_y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('gradient_x')) {
        this.gradient_x = initObj.gradient_x
      }
      else {
        this.gradient_x = [];
      }
      if (initObj.hasOwnProperty('gradient_y')) {
        this.gradient_y = initObj.gradient_y
      }
      else {
        this.gradient_y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type densityGradient
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [gradient_x]
    bufferOffset = _arraySerializer.float64(obj.gradient_x, buffer, bufferOffset, null);
    // Serialize message field [gradient_y]
    bufferOffset = _arraySerializer.float64(obj.gradient_y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type densityGradient
    let len;
    let data = new densityGradient(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [gradient_x]
    data.gradient_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [gradient_y]
    data.gradient_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.gradient_x.length;
    length += 8 * object.gradient_y.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/densityGradient';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbfc8681c266124a45b98ba27e25a5f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[] gradient_x
    float64[] gradient_y
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new densityGradient(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.gradient_x !== undefined) {
      resolved.gradient_x = msg.gradient_x;
    }
    else {
      resolved.gradient_x = []
    }

    if (msg.gradient_y !== undefined) {
      resolved.gradient_y = msg.gradient_y;
    }
    else {
      resolved.gradient_y = []
    }

    return resolved;
    }
};

module.exports = densityGradient;
