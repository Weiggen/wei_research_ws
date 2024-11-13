;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::TargetInfo)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'TargetInfo (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::TARGETINFO")
  (make-package "VORONOI_CBSA::TARGETINFO"))

(in-package "ROS")
;;//! \htmlinclude TargetInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass voronoi_cbsa::TargetInfo
  :super ros::object
  :slots (_id _position _height _covariance _weight _velocity _required_sensor ))

(defmethod voronoi_cbsa::TargetInfo
  (:init
   (&key
    ((:id __id) 0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:height __height) 0.0)
    ((:covariance __covariance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:weight __weight) 0.0)
    ((:velocity __velocity) (instance geometry_msgs::Twist :init))
    ((:required_sensor __required_sensor) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _position __position)
   (setq _height (float __height))
   (setq _covariance __covariance)
   (setq _weight (float __weight))
   (setq _velocity __velocity)
   (setq _required_sensor __required_sensor)
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
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:required_sensor
   (&optional __required_sensor)
   (if __required_sensor (setq _required_sensor __required_sensor)) _required_sensor)
  (:serialization-length
   ()
   (+
    ;; int64 _id
    8
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float32 _height
    4
    ;; float64[] _covariance
    (* 8    (length _covariance)) 4
    ;; float32 _weight
    4
    ;; geometry_msgs/Twist _velocity
    (send _velocity :serialization-length)
    ;; string[] _required_sensor
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _required_sensor)) 4
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
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64[] _covariance
     (write-long (length _covariance) s)
     (dotimes (i (length _covariance))
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float32 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Twist _velocity
       (send _velocity :serialize s)
     ;; string[] _required_sensor
     (write-long (length _required_sensor) s)
     (dolist (elem _required_sensor)
       (write-long (length elem) s) (princ elem s)
       )
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
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64[] _covariance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _covariance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float32 _weight
     (setq _weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Twist _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; string[] _required_sensor
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _required_sensor (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _required_sensor i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::TargetInfo :md5sum-) "a11d744703fb2fb21b1eb6c816c6cd6f")
(setf (get voronoi_cbsa::TargetInfo :datatype-) "voronoi_cbsa/TargetInfo")
(setf (get voronoi_cbsa::TargetInfo :definition-)
      "int64                   id
geometry_msgs/Point     position
float32                 height
float64[]               covariance
float32                 weight
geometry_msgs/Twist     velocity
string[]                required_sensor
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :voronoi_cbsa/TargetInfo "a11d744703fb2fb21b1eb6c816c6cd6f")


