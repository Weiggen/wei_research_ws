; Auto-generated. Do not edit!


(cl:in-package state_estimation-msg)


;//! \htmlinclude Plot.msg.html

(cl:defclass <Plot> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (GT_pose
    :reader GT_pose
    :initarg :GT_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (est_pose
    :reader est_pose
    :initarg :est_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (GT_twist
    :reader GT_twist
    :initarg :GT_twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (est_twist
    :reader est_twist
    :initarg :est_twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (RMSE_p
    :reader RMSE_p
    :initarg :RMSE_p
    :type cl:float
    :initform 0.0)
   (RMSE_v
    :reader RMSE_v
    :initarg :RMSE_v
    :type cl:float
    :initform 0.0)
   (det_p
    :reader det_p
    :initarg :det_p
    :type cl:float
    :initform 0.0))
)

(cl:defclass Plot (<Plot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimation-msg:<Plot> is deprecated: use state_estimation-msg:Plot instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:header-val is deprecated.  Use state_estimation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'GT_pose-val :lambda-list '(m))
(cl:defmethod GT_pose-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:GT_pose-val is deprecated.  Use state_estimation-msg:GT_pose instead.")
  (GT_pose m))

(cl:ensure-generic-function 'est_pose-val :lambda-list '(m))
(cl:defmethod est_pose-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:est_pose-val is deprecated.  Use state_estimation-msg:est_pose instead.")
  (est_pose m))

(cl:ensure-generic-function 'GT_twist-val :lambda-list '(m))
(cl:defmethod GT_twist-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:GT_twist-val is deprecated.  Use state_estimation-msg:GT_twist instead.")
  (GT_twist m))

(cl:ensure-generic-function 'est_twist-val :lambda-list '(m))
(cl:defmethod est_twist-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:est_twist-val is deprecated.  Use state_estimation-msg:est_twist instead.")
  (est_twist m))

(cl:ensure-generic-function 'RMSE_p-val :lambda-list '(m))
(cl:defmethod RMSE_p-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:RMSE_p-val is deprecated.  Use state_estimation-msg:RMSE_p instead.")
  (RMSE_p m))

(cl:ensure-generic-function 'RMSE_v-val :lambda-list '(m))
(cl:defmethod RMSE_v-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:RMSE_v-val is deprecated.  Use state_estimation-msg:RMSE_v instead.")
  (RMSE_v m))

(cl:ensure-generic-function 'det_p-val :lambda-list '(m))
(cl:defmethod det_p-val ((m <Plot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimation-msg:det_p-val is deprecated.  Use state_estimation-msg:det_p instead.")
  (det_p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plot>) ostream)
  "Serializes a message object of type '<Plot>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'GT_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'est_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'GT_twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'est_twist) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RMSE_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'RMSE_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'det_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plot>) istream)
  "Deserializes a message object of type '<Plot>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'GT_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'est_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'GT_twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'est_twist) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RMSE_p) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'RMSE_v) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'det_p) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plot>)))
  "Returns string type for a message object of type '<Plot>"
  "state_estimation/Plot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plot)))
  "Returns string type for a message object of type 'Plot"
  "state_estimation/Plot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plot>)))
  "Returns md5sum for a message object of type '<Plot>"
  "789b79485aceaa0b29456291c24a8393")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plot)))
  "Returns md5sum for a message object of type 'Plot"
  "789b79485aceaa0b29456291c24a8393")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plot>)))
  "Returns full string definition for message of type '<Plot>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose GT_pose~%geometry_msgs/Pose est_pose~%geometry_msgs/Twist GT_twist~%geometry_msgs/Twist est_twist~%float64 RMSE_p~%float64 RMSE_v~%float64 det_p~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plot)))
  "Returns full string definition for message of type 'Plot"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose GT_pose~%geometry_msgs/Pose est_pose~%geometry_msgs/Twist GT_twist~%geometry_msgs/Twist est_twist~%float64 RMSE_p~%float64 RMSE_v~%float64 det_p~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plot>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'GT_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'est_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'GT_twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'est_twist))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plot>))
  "Converts a ROS message object to a list"
  (cl:list 'Plot
    (cl:cons ':header (header msg))
    (cl:cons ':GT_pose (GT_pose msg))
    (cl:cons ':est_pose (est_pose msg))
    (cl:cons ':GT_twist (GT_twist msg))
    (cl:cons ':est_twist (est_twist msg))
    (cl:cons ':RMSE_p (RMSE_p msg))
    (cl:cons ':RMSE_v (RMSE_v msg))
    (cl:cons ':det_p (det_p msg))
))
