; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude ExchangeData.msg.html

(cl:defclass <ExchangeData> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (role
    :reader role
    :initarg :role
    :type voronoi_cbsa-msg:SensorArray
    :initform (cl:make-instance 'voronoi_cbsa-msg:SensorArray))
   (weights
    :reader weights
    :initarg :weights
    :type voronoi_cbsa-msg:WeightArray
    :initform (cl:make-instance 'voronoi_cbsa-msg:WeightArray))
   (sensor_scores
    :reader sensor_scores
    :initarg :sensor_scores
    :type voronoi_cbsa-msg:WeightArray
    :initform (cl:make-instance 'voronoi_cbsa-msg:WeightArray))
   (operation_range
    :reader operation_range
    :initarg :operation_range
    :type cl:float
    :initform 0.0)
   (approx_param
    :reader approx_param
    :initarg :approx_param
    :type cl:float
    :initform 0.0)
   (smoke_variance
    :reader smoke_variance
    :initarg :smoke_variance
    :type cl:float
    :initform 0.0)
   (camera_range
    :reader camera_range
    :initarg :camera_range
    :type cl:float
    :initform 0.0)
   (angle_of_view
    :reader angle_of_view
    :initarg :angle_of_view
    :type cl:float
    :initform 0.0)
   (camera_variance
    :reader camera_variance
    :initarg :camera_variance
    :type cl:float
    :initform 0.0)
   (vel_cmd
    :reader vel_cmd
    :initarg :vel_cmd
    :type std_msgs-msg:Float64MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float64MultiArray)))
)

(cl:defclass ExchangeData (<ExchangeData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExchangeData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExchangeData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<ExchangeData> is deprecated: use voronoi_cbsa-msg:ExchangeData instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:id-val is deprecated.  Use voronoi_cbsa-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:position-val is deprecated.  Use voronoi_cbsa-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'role-val :lambda-list '(m))
(cl:defmethod role-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:role-val is deprecated.  Use voronoi_cbsa-msg:role instead.")
  (role m))

(cl:ensure-generic-function 'weights-val :lambda-list '(m))
(cl:defmethod weights-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:weights-val is deprecated.  Use voronoi_cbsa-msg:weights instead.")
  (weights m))

(cl:ensure-generic-function 'sensor_scores-val :lambda-list '(m))
(cl:defmethod sensor_scores-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:sensor_scores-val is deprecated.  Use voronoi_cbsa-msg:sensor_scores instead.")
  (sensor_scores m))

(cl:ensure-generic-function 'operation_range-val :lambda-list '(m))
(cl:defmethod operation_range-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:operation_range-val is deprecated.  Use voronoi_cbsa-msg:operation_range instead.")
  (operation_range m))

(cl:ensure-generic-function 'approx_param-val :lambda-list '(m))
(cl:defmethod approx_param-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:approx_param-val is deprecated.  Use voronoi_cbsa-msg:approx_param instead.")
  (approx_param m))

(cl:ensure-generic-function 'smoke_variance-val :lambda-list '(m))
(cl:defmethod smoke_variance-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:smoke_variance-val is deprecated.  Use voronoi_cbsa-msg:smoke_variance instead.")
  (smoke_variance m))

(cl:ensure-generic-function 'camera_range-val :lambda-list '(m))
(cl:defmethod camera_range-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:camera_range-val is deprecated.  Use voronoi_cbsa-msg:camera_range instead.")
  (camera_range m))

(cl:ensure-generic-function 'angle_of_view-val :lambda-list '(m))
(cl:defmethod angle_of_view-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:angle_of_view-val is deprecated.  Use voronoi_cbsa-msg:angle_of_view instead.")
  (angle_of_view m))

(cl:ensure-generic-function 'camera_variance-val :lambda-list '(m))
(cl:defmethod camera_variance-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:camera_variance-val is deprecated.  Use voronoi_cbsa-msg:camera_variance instead.")
  (camera_variance m))

(cl:ensure-generic-function 'vel_cmd-val :lambda-list '(m))
(cl:defmethod vel_cmd-val ((m <ExchangeData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:vel_cmd-val is deprecated.  Use voronoi_cbsa-msg:vel_cmd instead.")
  (vel_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExchangeData>) ostream)
  "Serializes a message object of type '<ExchangeData>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'role) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'weights) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensor_scores) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'operation_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'approx_param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'smoke_variance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'camera_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle_of_view))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'camera_variance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel_cmd) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExchangeData>) istream)
  "Deserializes a message object of type '<ExchangeData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'role) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'weights) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensor_scores) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'operation_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'approx_param) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'smoke_variance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'camera_range) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_of_view) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'camera_variance) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel_cmd) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExchangeData>)))
  "Returns string type for a message object of type '<ExchangeData>"
  "voronoi_cbsa/ExchangeData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExchangeData)))
  "Returns string type for a message object of type 'ExchangeData"
  "voronoi_cbsa/ExchangeData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExchangeData>)))
  "Returns md5sum for a message object of type '<ExchangeData>"
  "e52bf14ec783bb82fb5c904ea32cc70b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExchangeData)))
  "Returns md5sum for a message object of type 'ExchangeData"
  "e52bf14ec783bb82fb5c904ea32cc70b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExchangeData>)))
  "Returns full string definition for message of type '<ExchangeData>"
  (cl:format cl:nil "int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%std_msgs/Float64MultiArray vel_cmd~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExchangeData)))
  "Returns full string definition for message of type 'ExchangeData"
  (cl:format cl:nil "int64               id~%geometry_msgs/Point position~%SensorArray         role~%WeightArray         weights~%WeightArray         sensor_scores~%float64             operation_range~%float64             approx_param~%float64             smoke_variance~%float64             camera_range~%float64             angle_of_view~%float64             camera_variance~%std_msgs/Float64MultiArray vel_cmd~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: voronoi_cbsa/SensorArray~%Sensor[] sensors~%================================================================================~%MSG: voronoi_cbsa/Sensor~%string type~%float64 score~%================================================================================~%MSG: voronoi_cbsa/WeightArray~%Weight[] weights~%================================================================================~%MSG: voronoi_cbsa/Weight~%string  type~%int16   event_id~%float64 score~%================================================================================~%MSG: std_msgs/Float64MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float64[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExchangeData>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'role))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'weights))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensor_scores))
     8
     8
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel_cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExchangeData>))
  "Converts a ROS message object to a list"
  (cl:list 'ExchangeData
    (cl:cons ':id (id msg))
    (cl:cons ':position (position msg))
    (cl:cons ':role (role msg))
    (cl:cons ':weights (weights msg))
    (cl:cons ':sensor_scores (sensor_scores msg))
    (cl:cons ':operation_range (operation_range msg))
    (cl:cons ':approx_param (approx_param msg))
    (cl:cons ':smoke_variance (smoke_variance msg))
    (cl:cons ':camera_range (camera_range msg))
    (cl:cons ':angle_of_view (angle_of_view msg))
    (cl:cons ':camera_variance (camera_variance msg))
    (cl:cons ':vel_cmd (vel_cmd msg))
))
