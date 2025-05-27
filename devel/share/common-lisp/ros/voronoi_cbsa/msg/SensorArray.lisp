; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude SensorArray.msg.html

(cl:defclass <SensorArray> (roslisp-msg-protocol:ros-message)
  ((sensors
    :reader sensors
    :initarg :sensors
    :type (cl:vector voronoi_cbsa-msg:Sensor)
   :initform (cl:make-array 0 :element-type 'voronoi_cbsa-msg:Sensor :initial-element (cl:make-instance 'voronoi_cbsa-msg:Sensor))))
)

(cl:defclass SensorArray (<SensorArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<SensorArray> is deprecated: use voronoi_cbsa-msg:SensorArray instead.")))

(cl:ensure-generic-function 'sensors-val :lambda-list '(m))
(cl:defmethod sensors-val ((m <SensorArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:sensors-val is deprecated.  Use voronoi_cbsa-msg:sensors instead.")
  (sensors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorArray>) ostream)
  "Serializes a message object of type '<SensorArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sensors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorArray>) istream)
  "Deserializes a message object of type '<SensorArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voronoi_cbsa-msg:Sensor))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorArray>)))
  "Returns string type for a message object of type '<SensorArray>"
  "voronoi_cbsa/SensorArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorArray)))
  "Returns string type for a message object of type 'SensorArray"
  "voronoi_cbsa/SensorArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorArray>)))
  "Returns md5sum for a message object of type '<SensorArray>"
  "e1d908c1ca577e30068931de829be081")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorArray)))
  "Returns md5sum for a message object of type 'SensorArray"
  "e1d908c1ca577e30068931de829be081")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorArray>)))
  "Returns full string definition for message of type '<SensorArray>"
  (cl:format cl:nil "Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorArray)))
  "Returns full string definition for message of type 'SensorArray"
  (cl:format cl:nil "Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorArray>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorArray
    (cl:cons ':sensors (sensors msg))
))
