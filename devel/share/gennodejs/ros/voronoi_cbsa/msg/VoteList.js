// Auto-generated. Do not edit!

// (in-package voronoi_cbsa.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class VoteList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.vote = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('vote')) {
        this.vote = initObj.vote
      }
      else {
        this.vote = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VoteList
    // Serialize message field [index]
    bufferOffset = _serializer.int64(obj.index, buffer, bufferOffset);
    // Serialize message field [vote]
    bufferOffset = _serializer.bool(obj.vote, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VoteList
    let len;
    let data = new VoteList(null);
    // Deserialize message field [index]
    data.index = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [vote]
    data.vote = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_cbsa/VoteList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4974848ca173ad3d0044de0990068ffc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 index
    bool vote
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VoteList(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.vote !== undefined) {
      resolved.vote = msg.vote;
    }
    else {
      resolved.vote = false
    }

    return resolved;
    }
};

module.exports = VoteList;
