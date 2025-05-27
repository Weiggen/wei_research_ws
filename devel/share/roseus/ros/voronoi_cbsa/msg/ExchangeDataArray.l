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

(setf (get voronoi_cbsa::ExchangeDataArray :md5sum-) "18f160db55a6039398f060d660965da4")
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
geometry_msgs/Point velocity

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
")



(provide :voronoi_cbsa/ExchangeDataArray "18f160db55a6039398f060d660965da4")


