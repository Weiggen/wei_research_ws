; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude TargetInfoArray.msg.html

(cl:defclass <TargetInfoArray> (roslisp-msg-protocol:ros-message)
  ((targets
    :reader targets
    :initarg :targets
    :type (cl:vector voronoi_cbsa-msg:TargetInfo)
   :initform (cl:make-array 0 :element-type 'voronoi_cbsa-msg:TargetInfo :initial-element (cl:make-instance 'voronoi_cbsa-msg:TargetInfo))))
)

(cl:defclass TargetInfoArray (<TargetInfoArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TargetInfoArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TargetInfoArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<TargetInfoArray> is deprecated: use voronoi_cbsa-msg:TargetInfoArray instead.")))

(cl:ensure-generic-function 'targets-val :lambda-list '(m))
(cl:defmethod targets-val ((m <TargetInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:targets-val is deprecated.  Use voronoi_cbsa-msg:targets instead.")
  (targets m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TargetInfoArray>) ostream)
  "Serializes a message object of type '<TargetInfoArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'targets))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'targets))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TargetInfoArray>) istream)
  "Deserializes a message object of type '<TargetInfoArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'targets) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'targets)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voronoi_cbsa-msg:TargetInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TargetInfoArray>)))
  "Returns string type for a message object of type '<TargetInfoArray>"
  "voronoi_cbsa/TargetInfoArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TargetInfoArray)))
  "Returns string type for a message object of type 'TargetInfoArray"
  "voronoi_cbsa/TargetInfoArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TargetInfoArray>)))
  "Returns md5sum for a message object of type '<TargetInfoArray>"
  "cb14593f2221c85378b0d6d6b83f5b15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TargetInfoArray)))
  "Returns md5sum for a message object of type 'TargetInfoArray"
  "cb14593f2221c85378b0d6d6b83f5b15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TargetInfoArray>)))
  "Returns full string definition for message of type '<TargetInfoArray>"
  (cl:format cl:nil "TargetInfo[] targets~%================================================================================~%MSG: voronoi_cbsa/TargetInfo~%int64                   id~%geometry_msgs/Point     position~%float32                 height~%float64[]               covariance~%float32                 weight~%geometry_msgs/Twist     velocity~%string[]                required_sensor~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TargetInfoArray)))
  "Returns full string definition for message of type 'TargetInfoArray"
  (cl:format cl:nil "TargetInfo[] targets~%================================================================================~%MSG: voronoi_cbsa/TargetInfo~%int64                   id~%geometry_msgs/Point     position~%float32                 height~%float64[]               covariance~%float32                 weight~%geometry_msgs/Twist     velocity~%string[]                required_sensor~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TargetInfoArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'targets) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TargetInfoArray>))
  "Converts a ROS message object to a list"
  (cl:list 'TargetInfoArray
    (cl:cons ':targets (targets msg))
))
