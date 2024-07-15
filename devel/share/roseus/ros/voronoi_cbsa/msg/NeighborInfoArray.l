;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::NeighborInfoArray)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'NeighborInfoArray (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::NEIGHBORINFOARRAY")
  (make-package "VORONOI_CBSA::NEIGHBORINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude NeighborInfoArray.msg.html


(defclass voronoi_cbsa::NeighborInfoArray
  :super ros::object
  :slots (_neighbors ))

(defmethod voronoi_cbsa::NeighborInfoArray
  (:init
   (&key
    ((:neighbors __neighbors) ())
    )
   (send-super :init)
   (setq _neighbors __neighbors)
   self)
  (:neighbors
   (&rest __neighbors)
   (if (keywordp (car __neighbors))
       (send* _neighbors __neighbors)
     (progn
       (if __neighbors (setq _neighbors (car __neighbors)))
       _neighbors)))
  (:serialization-length
   ()
   (+
    ;; voronoi_cbsa/NeighborInfo[] _neighbors
    (apply #'+ (send-all _neighbors :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; voronoi_cbsa/NeighborInfo[] _neighbors
     (write-long (length _neighbors) s)
     (dolist (elem _neighbors)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; voronoi_cbsa/NeighborInfo[] _neighbors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _neighbors (let (r) (dotimes (i n) (push (instance voronoi_cbsa::NeighborInfo :init) r)) r))
     (dolist (elem- _neighbors)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::NeighborInfoArray :md5sum-) "dc1d874a9451abd6cbb97a2b631e4d4f")
(setf (get voronoi_cbsa::NeighborInfoArray :datatype-) "voronoi_cbsa/NeighborInfoArray")
(setf (get voronoi_cbsa::NeighborInfoArray :definition-)
      "NeighborInfo[] neighbors
================================================================================
MSG: voronoi_cbsa/NeighborInfo
int16 id
geometry_msgs/Point position
SensorArray role
float64             operation_range
float64             approx_param
float64             smoke_variance
float64             camera_range
float64             angle_of_view
float64             camera_variance
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
")



(provide :voronoi_cbsa/NeighborInfoArray "dc1d874a9451abd6cbb97a2b631e4d4f")


