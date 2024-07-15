;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::ExchangeData)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'ExchangeData (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::EXCHANGEDATA")
  (make-package "VORONOI_CBSA::EXCHANGEDATA"))

(in-package "ROS")
;;//! \htmlinclude ExchangeData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass voronoi_cbsa::ExchangeData
  :super ros::object
  :slots (_id _position _role _weights _sensor_scores _operation_range _approx_param _smoke_variance _camera_range _angle_of_view _camera_variance ))

(defmethod voronoi_cbsa::ExchangeData
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:role __role) (instance voronoi_cbsa::SensorArray :init))
    ((:weights __weights) (instance voronoi_cbsa::WeightArray :init))
    ((:sensor_scores __sensor_scores) (instance voronoi_cbsa::WeightArray :init))
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
   (setq _weights __weights)
   (setq _sensor_scores __sensor_scores)
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
  (:weights
   (&rest __weights)
   (if (keywordp (car __weights))
       (send* _weights __weights)
     (progn
       (if __weights (setq _weights (car __weights)))
       _weights)))
  (:sensor_scores
   (&rest __sensor_scores)
   (if (keywordp (car __sensor_scores))
       (send* _sensor_scores __sensor_scores)
     (progn
       (if __sensor_scores (setq _sensor_scores (car __sensor_scores)))
       _sensor_scores)))
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
    ;; int64 _id
    8
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; voronoi_cbsa/SensorArray _role
    (send _role :serialization-length)
    ;; voronoi_cbsa/WeightArray _weights
    (send _weights :serialization-length)
    ;; voronoi_cbsa/WeightArray _sensor_scores
    (send _sensor_scores :serialization-length)
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
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; voronoi_cbsa/SensorArray _role
       (send _role :serialize s)
     ;; voronoi_cbsa/WeightArray _weights
       (send _weights :serialize s)
     ;; voronoi_cbsa/WeightArray _sensor_scores
       (send _sensor_scores :serialize s)
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
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; voronoi_cbsa/SensorArray _role
     (send _role :deserialize buf ptr-) (incf ptr- (send _role :serialization-length))
   ;; voronoi_cbsa/WeightArray _weights
     (send _weights :deserialize buf ptr-) (incf ptr- (send _weights :serialization-length))
   ;; voronoi_cbsa/WeightArray _sensor_scores
     (send _sensor_scores :deserialize buf ptr-) (incf ptr- (send _sensor_scores :serialization-length))
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

(setf (get voronoi_cbsa::ExchangeData :md5sum-) "77f411baa5b1143c9a101ed6a3425781")
(setf (get voronoi_cbsa::ExchangeData :datatype-) "voronoi_cbsa/ExchangeData")
(setf (get voronoi_cbsa::ExchangeData :definition-)
      "int64               id
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



(provide :voronoi_cbsa/ExchangeData "77f411baa5b1143c9a101ed6a3425781")


