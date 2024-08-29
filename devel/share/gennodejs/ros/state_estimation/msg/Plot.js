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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Plot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.GT_pose = null;
      this.est_pose = null;
      this.GT_twist = null;
      this.est_twist = null;
      this.RMSE_p = null;
      this.RMSE_v = null;
      this.det_p = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('GT_pose')) {
        this.GT_pose = initObj.GT_pose
      }
      else {
        this.GT_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('est_pose')) {
        this.est_pose = initObj.est_pose
      }
      else {
        this.est_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('GT_twist')) {
        this.GT_twist = initObj.GT_twist
      }
      else {
        this.GT_twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('est_twist')) {
        this.est_twist = initObj.est_twist
      }
      else {
        this.est_twist = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('RMSE_p')) {
        this.RMSE_p = initObj.RMSE_p
      }
      else {
        this.RMSE_p = 0.0;
      }
      if (initObj.hasOwnProperty('RMSE_v')) {
        this.RMSE_v = initObj.RMSE_v
      }
      else {
        this.RMSE_v = 0.0;
      }
      if (initObj.hasOwnProperty('det_p')) {
        this.det_p = initObj.det_p
      }
      else {
        this.det_p = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Plot
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [GT_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.GT_pose, buffer, bufferOffset);
    // Serialize message field [est_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.est_pose, buffer, bufferOffset);
    // Serialize message field [GT_twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.GT_twist, buffer, bufferOffset);
    // Serialize message field [est_twist]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.est_twist, buffer, bufferOffset);
    // Serialize message field [RMSE_p]
    bufferOffset = _serializer.float64(obj.RMSE_p, buffer, bufferOffset);
    // Serialize message field [RMSE_v]
    bufferOffset = _serializer.float64(obj.RMSE_v, buffer, bufferOffset);
    // Serialize message field [det_p]
    bufferOffset = _serializer.float64(obj.det_p, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Plot
    let len;
    let data = new Plot(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [GT_pose]
    data.GT_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [est_pose]
    data.est_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [GT_twist]
    data.GT_twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [est_twist]
    data.est_twist = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [RMSE_p]
    data.RMSE_p = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [RMSE_v]
    data.RMSE_v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [det_p]
    data.det_p = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 232;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimation/Plot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '789b79485aceaa0b29456291c24a8393';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Pose GT_pose
    geometry_msgs/Pose est_pose
    geometry_msgs/Twist GT_twist
    geometry_msgs/Twist est_twist
    float64 RMSE_p
    float64 RMSE_v
    float64 det_p
    
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
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
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
    const resolved = new Plot(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.GT_pose !== undefined) {
      resolved.GT_pose = geometry_msgs.msg.Pose.Resolve(msg.GT_pose)
    }
    else {
      resolved.GT_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.est_pose !== undefined) {
      resolved.est_pose = geometry_msgs.msg.Pose.Resolve(msg.est_pose)
    }
    else {
      resolved.est_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.GT_twist !== undefined) {
      resolved.GT_twist = geometry_msgs.msg.Twist.Resolve(msg.GT_twist)
    }
    else {
      resolved.GT_twist = new geometry_msgs.msg.Twist()
    }

    if (msg.est_twist !== undefined) {
      resolved.est_twist = geometry_msgs.msg.Twist.Resolve(msg.est_twist)
    }
    else {
      resolved.est_twist = new geometry_msgs.msg.Twist()
    }

    if (msg.RMSE_p !== undefined) {
      resolved.RMSE_p = msg.RMSE_p;
    }
    else {
      resolved.RMSE_p = 0.0
    }

    if (msg.RMSE_v !== undefined) {
      resolved.RMSE_v = msg.RMSE_v;
    }
    else {
      resolved.RMSE_v = 0.0
    }

    if (msg.det_p !== undefined) {
      resolved.det_p = msg.det_p;
    }
    else {
      resolved.det_p = 0.0
    }

    return resolved;
    }
};

module.exports = Plot;
