;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::TargetInfoArray)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'TargetInfoArray (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::TARGETINFOARRAY")
  (make-package "VORONOI_CBSA::TARGETINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude TargetInfoArray.msg.html


(defclass voronoi_cbsa::TargetInfoArray
  :super ros::object
  :slots (_targets ))

(defmethod voronoi_cbsa::TargetInfoArray
  (:init
   (&key
    ((:targets __targets) ())
    )
   (send-super :init)
   (setq _targets __targets)
   self)
  (:targets
   (&rest __targets)
   (if (keywordp (car __targets))
       (send* _targets __targets)
     (progn
       (if __targets (setq _targets (car __targets)))
       _targets)))
  (:serialization-length
   ()
   (+
    ;; voronoi_cbsa/TargetInfo[] _targets
    (apply #'+ (send-all _targets :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; voronoi_cbsa/TargetInfo[] _targets
     (write-long (length _targets) s)
     (dolist (elem _targets)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; voronoi_cbsa/TargetInfo[] _targets
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _targets (let (r) (dotimes (i n) (push (instance voronoi_cbsa::TargetInfo :init) r)) r))
     (dolist (elem- _targets)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::TargetInfoArray :md5sum-) "cb14593f2221c85378b0d6d6b83f5b15")
(setf (get voronoi_cbsa::TargetInfoArray :datatype-) "voronoi_cbsa/TargetInfoArray")
(setf (get voronoi_cbsa::TargetInfoArray :definition-)
      "TargetInfo[] targets
================================================================================
MSG: voronoi_cbsa/TargetInfo
int64                   id
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



(provide :voronoi_cbsa/TargetInfoArray "cb14593f2221c85378b0d6d6b83f5b15")


