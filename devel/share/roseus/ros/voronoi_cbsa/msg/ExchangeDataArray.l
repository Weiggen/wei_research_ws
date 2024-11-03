;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::ExchangeDataArray)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'ExchangeDataArray (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::EXCHANGEDATAARRAY")
  (make-package "VORONOI_CBSA::EXCHANGEDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude ExchangeDataArray.msg.html


(defclass voronoi_cbsa::ExchangeDataArray
  :super ros::object
  :slots (_data ))

(defmethod voronoi_cbsa::ExchangeDataArray
  (:init
   (&key
    ((:data __data) ())
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; voronoi_cbsa/ExchangeData[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; voronoi_cbsa/ExchangeData[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; voronoi_cbsa/ExchangeData[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance voronoi_cbsa::ExchangeData :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::ExchangeDataArray :md5sum-) "123206d5e1536f22170e5c5739052f99")
(setf (get voronoi_cbsa::ExchangeDataArray :datatype-) "voronoi_cbsa/ExchangeDataArray")
(setf (get voronoi_cbsa::ExchangeDataArray :definition-)
      "ExchangeData[] data
================================================================================
MSG: voronoi_cbsa/ExchangeData
int64               id
geometry_msgs/Point position
SensorArray         role
WeightArray         weights
WeightArray         sensor_scores
float64             operation_range
float64             approx_param
float64             smoke_variance
float64             camera_range
float64             angle_of_view
float64             camera_variance
std_msgs/Float64MultiArray vel_cmd

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: voronoi_cbsa/SensorArray
Sensor[] sensors
================================================================================
MSG: voronoi_cbsa/Sensor
string type
float64 score
================================================================================
MSG: voronoi_cbsa/WeightArray
Weight[] weights
================================================================================
MSG: voronoi_cbsa/Weight
string  type
int16   event_id
float64 score
================================================================================
MSG: std_msgs/Float64MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float64[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :voronoi_cbsa/ExchangeDataArray "123206d5e1536f22170e5c5739052f99")


