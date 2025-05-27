; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude ExchangeDataArray.msg.html

(cl:defclass <ExchangeDataArray> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector voronoi_cbsa-msg:ExchangeData)
   :initform (cl:make-array 0 :element-type 'voronoi_cbsa-msg:ExchangeData :initial-element (cl:make-instance 'voronoi_cbsa-msg:ExchangeData))))
)

(cl:defclass ExchangeDataArray (<ExchangeDataArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExchangeDataArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExchangeDataArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<ExchangeDataArray> is deprecated: use voronoi_cbsa-msg:ExchangeDataArray instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ExchangeDataArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:data-val is deprecated.  Use voronoi_cbsa-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExchangeDataArray>) ostream)
  "Serializes a message object of type '<ExchangeDataArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExchangeDataArray>) istream)
  "Deserializes a message object of type '<ExchangeDataArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voronoi_cbsa-msg:ExchangeData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExchangeDataArray>)))
  "Returns string type for a message object of type '<ExchangeDataArray>"
  "voronoi_cbsa/ExchangeDataArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExchangeDataArray)))
  "Returns string type for a message object of type 'ExchangeDataArray"
  "voronoi_cbsa/ExchangeDataArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExchangeDataArray>)))
  "Returns md5sum for a message object of type '<ExchangeDataArray>"
  "18f160db55a6039398f060d660965da4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExchangeDataArray)))
  "Returns md5sum for a message object of type 'ExchangeDataArray"
  "18f160db55a6039398f060d660965da4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExchangeDataArray>)))
  "Returns full string definition for message of type '<ExchangeDataArray>"
  (cl:format cl:nil "ExchangeData[] data~%================================================================================~%MSG: voronoi_cbsa/ExchangeData~%int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%geometry_msgs/Point velocity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExchangeDataArray)))
  "Returns full string definition for message of type 'ExchangeDataArray"
  (cl:format cl:nil "ExchangeData[] data~%================================================================================~%MSG: voronoi_cbsa/ExchangeData~%int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%geometry_msgs/Point velocity~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExchangeDataArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExchangeDataArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ExchangeDataArray
    (cl:cons ':data (data msg))
))
