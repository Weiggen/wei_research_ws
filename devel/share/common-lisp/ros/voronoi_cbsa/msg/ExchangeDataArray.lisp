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
  "123206d5e1536f22170e5c5739052f99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExchangeDataArray)))
  "Returns md5sum for a message object of type 'ExchangeDataArray"
  "123206d5e1536f22170e5c5739052f99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExchangeDataArray>)))
  "Returns full string definition for message of type '<ExchangeDataArray>"
  (cl:format cl:nil "ExchangeData[] data~%================================================================================~%MSG: voronoi_cbsa/ExchangeData~%int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%std_msgs/Float64MultiArray vel_cmd~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExchangeDataArray)))
  "Returns full string definition for message of type 'ExchangeDataArray"
  (cl:format cl:nil "ExchangeData[] data~%================================================================================~%MSG: voronoi_cbsa/ExchangeData~%int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%std_msgs/Float64MultiArray vel_cmd~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExchangeDataArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExchangeDataArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ExchangeDataArray
    (cl:cons ':data (data msg))
))
