;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::NeighborInfo)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'NeighborInfo (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::NEIGHBORINFO")
  (make-package "VORONOI_CBSA::NEIGHBORINFO"))

(in-package "ROS")
;;//! \htmlinclude NeighborInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass voronoi_cbsa::NeighborInfo
  :super ros::object
  :slots (_id _position _role _operation_range _approx_param _smoke_variance _camera_range _angle_of_view _camera_variance ))

(defmethod voronoi_cbsa::NeighborInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:role __role) (instance voronoi_cbsa::SensorArray :init))
    ((:operation_range __operation_range) 0.0)
    ((:approx_param __approx_param) 0.0)
    ((:smoke_variance __smoke_variance) 0.0)
    ((:camera_range __camera_range) 0.0)
    ((:angle_of_view __angle_of_view) 0.0)
    ((:camera_variance __camera_variance) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position __position)
   (setq _role __role)
   (setq _operation_range (float __operation_range))
   (setq _approx_param (float __approx_param))
   (setq _smoke_variance (float __smoke_variance))
   (setq _camera_range (float __camera_range))
   (setq _angle_of_view (float __angle_of_view))
   (setq _camera_variance (float __camera_variance))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:role
   (&rest __role)
   (if (keywordp (car __role))
       (send* _role __role)
     (progn
       (if __role (setq _role (car __role)))
       _role)))
  (:operation_range
   (&optional __operation_range)
   (if __operation_range (setq _operation_range __operation_range)) _operation_range)
  (:approx_param
   (&optional __approx_param)
   (if __approx_param (setq _approx_param __approx_param)) _approx_param)
  (:smoke_variance
   (&optional __smoke_variance)
   (if __smoke_variance (setq _smoke_variance __smoke_variance)) _smoke_variance)
  (:camera_range
   (&optional __camera_range)
   (if __camera_range (setq _camera_range __camera_range)) _camera_range)
  (:angle_of_view
   (&optional __angle_of_view)
   (if __angle_of_view (setq _angle_of_view __angle_of_view)) _angle_of_view)
  (:camera_variance
   (&optional __camera_variance)
   (if __camera_variance (setq _camera_variance __camera_variance)) _camera_variance)
  (:serialization-length
   ()
   (+
    ;; int16 _id
    2
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; voronoi_cbsa/SensorArray _role
    (send _role :serialization-length)
    ;; float64 _operation_range
    8
    ;; float64 _approx_param
    8
    ;; float64 _smoke_variance
    8
    ;; float64 _camera_range
    8
    ;; float64 _angle_of_view
    8
    ;; float64 _camera_variance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _id
       (write-word _id s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; voronoi_cbsa/SensorArray _role
       (send _role :serialize s)
     ;; float64 _operation_range
       (sys::poke _operation_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _approx_param
       (sys::poke _approx_param (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _smoke_variance
       (sys::poke _smoke_variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _camera_range
       (sys::poke _camera_range (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle_of_view
       (sys::poke _angle_of_view (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _camera_variance
       (sys::poke _camera_variance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; voronoi_cbsa/SensorArray _role
     (send _role :deserialize buf ptr-) (incf ptr- (send _role :serialization-length))
   ;; float64 _operation_range
     (setq _operation_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _approx_param
     (setq _approx_param (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _smoke_variance
     (setq _smoke_variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _camera_range
     (setq _camera_range (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle_of_view
     (setq _angle_of_view (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _camera_variance
     (setq _camera_variance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get voronoi_cbsa::NeighborInfo :md5sum-) "11f2e8ce0512f33d5dfa93d67585eb65")
(setf (get voronoi_cbsa::NeighborInfo :datatype-) "voronoi_cbsa/NeighborInfo")
(setf (get voronoi_cbsa::NeighborInfo :definition-)
      "int16 id
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



(provide :voronoi_cbsa/NeighborInfo "11f2e8ce0512f33d5dfa93d67585eb65")


