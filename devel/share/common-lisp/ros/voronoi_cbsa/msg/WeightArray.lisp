; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude WeightArray.msg.html

(cl:defclass <WeightArray> (roslisp-msg-protocol:ros-message)
  ((weights
    :reader weights
    :initarg :weights
    :type (cl:vector voronoi_cbsa-msg:Weight)
   :initform (cl:make-array 0 :element-type 'voronoi_cbsa-msg:Weight :initial-element (cl:make-instance 'voronoi_cbsa-msg:Weight))))
)

(cl:defclass WeightArray (<WeightArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WeightArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WeightArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<WeightArray> is deprecated: use voronoi_cbsa-msg:WeightArray instead.")))

(cl:ensure-generic-function 'weights-val :lambda-list '(m))
(cl:defmethod weights-val ((m <WeightArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:weights-val is deprecated.  Use voronoi_cbsa-msg:weights instead.")
  (weights m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WeightArray>) ostream)
  "Serializes a message object of type '<WeightArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'weights))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'weights))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WeightArray>) istream)
  "Deserializes a message object of type '<WeightArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'weights) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'weights)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voronoi_cbsa-msg:Weight))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WeightArray>)))
  "Returns string type for a message object of type '<WeightArray>"
  "voronoi_cbsa/WeightArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WeightArray)))
  "Returns string type for a message object of type 'WeightArray"
  "voronoi_cbsa/WeightArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WeightArray>)))
  "Returns md5sum for a message object of type '<WeightArray>"
  "a52d516a208ee351d816a3ad44a096ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WeightArray)))
  "Returns md5sum for a message object of type 'WeightArray"
  "a52d516a208ee351d816a3ad44a096ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WeightArray>)))
  "Returns full string definition for message of type '<WeightArray>"
  (cl:format cl:nil "Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WeightArray)))
  "Returns full string definition for message of type 'WeightArray"
  (cl:format cl:nil "Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WeightArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'weights) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WeightArray>))
  "Converts a ROS message object to a list"
  (cl:list 'WeightArray
    (cl:cons ':weights (weights msg))
))
