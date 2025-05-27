// Auto-generated. Do not edit!

// (in-package state_estimation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EIFpairStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.stateSize = null;
      this.id = null;
      this.P_hat = null;
      this.X_hat = null;
      this.s = null;
      this.y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('stateSize')) {
        this.stateSize = initObj.stateSize
      }
      else {
        this.stateSize = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('P_hat')) {
        this.P_hat = initObj.P_hat
      }
      else {
        this.P_hat = [];
      }
      if (initObj.hasOwnProperty('X_hat')) {
        this.X_hat = initObj.X_hat
      }
      else {
        this.X_hat = [];
      }
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EIFpairStamped
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [stateSize]
    bufferOffset = _serializer.int32(obj.stateSize, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [P_hat]
    bufferOffset = _arraySerializer.float64(obj.P_hat, buffer, bufferOffset, null);
    // Serialize message field [X_hat]
    bufferOffset = _arraySerializer.float64(obj.X_hat, buffer, bufferOffset, null);
    // Serialize message field [s]
    bufferOffset = _arraySerializer.float64(obj.s, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float64(obj.y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EIFpairStamped
    let len;
    let data = new EIFpairStamped(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [stateSize]
    data.stateSize = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [P_hat]
    data.P_hat = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [X_hat]
    data.X_hat = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [s]
    data.s = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.P_hat.length;
    length += 8 * object.X_hat.length;
    length += 8 * object.s.length;
    length += 8 * object.y.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimation/EIFpairStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26b414ae046293c626e824119402b4fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 stateSize
    int32 id
    float64[] P_hat
    float64[] X_hat
    float64[] s
    float64[] y
    
    
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
    const resolved = new EIFpairStamped(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.stateSize !== undefined) {
      resolved.stateSize = msg.stateSize;
    }
    else {
      resolved.stateSize = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.P_hat !== undefined) {
      resolved.P_hat = msg.P_hat;
    }
    else {
      resolved.P_hat = []
    }

    if (msg.X_hat !== undefined) {
      resolved.X_hat = msg.X_hat;
    }
    else {
      resolved.X_hat = []
    }

    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    return resolved;
    }
};

module.exports = EIFpairStamped;
