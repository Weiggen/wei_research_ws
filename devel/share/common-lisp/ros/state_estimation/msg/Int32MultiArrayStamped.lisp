; Auto-generated. Do not edit!


(cl:in-package state_estimation-msg)


;//! \htmlinclude Int32MultiArrayStamped.msg.html

(cl:defclass <Int32MultiArrayStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass Int32MultiArrayStamped (<Int32MultiArrayStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int32MultiArrayStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int32MultiArrayStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimation-msg:<Int32MultiArrayStamped> is deprecated: use state_estimation-msg:Int32MultiArrayStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Int32MultiArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:header-val is deprecated.  Use state_estimation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Int32MultiArrayStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:data-val is deprecated.  Use state_estimation-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int32MultiArrayStamped>) ostream)
  "Serializes a message object of type '<Int32MultiArrayStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int32MultiArrayStamped>) istream)
  "Deserializes a message object of type '<Int32MultiArrayStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int32MultiArrayStamped>)))
  "Returns string type for a message object of type '<Int32MultiArrayStamped>"
  "state_estimation/Int32MultiArrayStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int32MultiArrayStamped)))
  "Returns string type for a message object of type 'Int32MultiArrayStamped"
  "state_estimation/Int32MultiArrayStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int32MultiArrayStamped>)))
  "Returns md5sum for a message object of type '<Int32MultiArrayStamped>"
  "1ce4762ce13f3d9e1f17586acc253067")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int32MultiArrayStamped)))
  "Returns md5sum for a message object of type 'Int32MultiArrayStamped"
  "1ce4762ce13f3d9e1f17586acc253067")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int32MultiArrayStamped>)))
  "Returns full string definition for message of type '<Int32MultiArrayStamped>"
  (cl:format cl:nil "Header header~%int32[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int32MultiArrayStamped)))
  "Returns full string definition for message of type 'Int32MultiArrayStamped"
  (cl:format cl:nil "Header header~%int32[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int32MultiArrayStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int32MultiArrayStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'Int32MultiArrayStamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
