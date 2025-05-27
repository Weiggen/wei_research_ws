; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude NeighborInfoArray.msg.html

(cl:defclass <NeighborInfoArray> (roslisp-msg-protocol:ros-message)
  ((neighbors
    :reader neighbors
    :initarg :neighbors
    :type (cl:vector voronoi_cbsa-msg:NeighborInfo)
   :initform (cl:make-array 0 :element-type 'voronoi_cbsa-msg:NeighborInfo :initial-element (cl:make-instance 'voronoi_cbsa-msg:NeighborInfo))))
)

(cl:defclass NeighborInfoArray (<NeighborInfoArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NeighborInfoArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NeighborInfoArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<NeighborInfoArray> is deprecated: use voronoi_cbsa-msg:NeighborInfoArray instead.")))

(cl:ensure-generic-function 'neighbors-val :lambda-list '(m))
(cl:defmethod neighbors-val ((m <NeighborInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:neighbors-val is deprecated.  Use voronoi_cbsa-msg:neighbors instead.")
  (neighbors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NeighborInfoArray>) ostream)
  "Serializes a message object of type '<NeighborInfoArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'neighbors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'neighbors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NeighborInfoArray>) istream)
  "Deserializes a message object of type '<NeighborInfoArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'neighbors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'neighbors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voronoi_cbsa-msg:NeighborInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NeighborInfoArray>)))
  "Returns string type for a message object of type '<NeighborInfoArray>"
  "voronoi_cbsa/NeighborInfoArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NeighborInfoArray)))
  "Returns string type for a message object of type 'NeighborInfoArray"
  "voronoi_cbsa/NeighborInfoArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NeighborInfoArray>)))
  "Returns md5sum for a message object of type '<NeighborInfoArray>"
  "dc1d874a9451abd6cbb97a2b631e4d4f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NeighborInfoArray)))
  "Returns md5sum for a message object of type 'NeighborInfoArray"
  "dc1d874a9451abd6cbb97a2b631e4d4f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NeighborInfoArray>)))
  "Returns full string definition for message of type '<NeighborInfoArray>"
  (cl:format cl:nil "NeighborInfo[] neighbors~%================================================================================~%MSG: voronoi_cbsa/NeighborInfo~%int16 id~%geometry_msgs/Point position~%SensorArray role~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NeighborInfoArray)))
  "Returns full string definition for message of type 'NeighborInfoArray"
  (cl:format cl:nil "NeighborInfo[] neighbors~%================================================================================~%MSG: voronoi_cbsa/NeighborInfo~%int16 id~%geometry_msgs/Point position~%SensorArray role~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NeighborInfoArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'neighbors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NeighborInfoArray>))
  "Converts a ROS message object to a list"
  (cl:list 'NeighborInfoArray
    (cl:cons ':neighbors (neighbors msg))
))
