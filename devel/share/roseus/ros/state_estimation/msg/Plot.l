;; Auto-generated. Do not edit!


(when (boundp 'state_estimation::Plot)
  (if (not (find-package "STATE_ESTIMATION"))
    (make-package "STATE_ESTIMATION"))
  (shadow 'Plot (find-package "STATE_ESTIMATION")))
(unless (find-package "STATE_ESTIMATION::PLOT")
  (make-package "STATE_ESTIMATION::PLOT"))

(in-package "ROS")
;;//! \htmlinclude Plot.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimation::Plot
  :super ros::object
  :slots (_header _GT_pose _est_pose _GT_twist _est_twist _RMSE_p _RMSE_v _det_p ))

(defmethod state_estimation::Plot
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:GT_pose __GT_pose) (instance geometry_msgs::Pose :init))
    ((:est_pose __est_pose) (instance geometry_msgs::Pose :init))
    ((:GT_twist __GT_twist) (instance geometry_msgs::Twist :init))
    ((:est_twist __est_twist) (instance geometry_msgs::Twist :init))
    ((:RMSE_p __RMSE_p) 0.0)
    ((:RMSE_v __RMSE_v) 0.0)
    ((:det_p __det_p) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _GT_pose __GT_pose)
   (setq _est_pose __est_pose)
   (setq _GT_twist __GT_twist)
   (setq _est_twist __est_twist)
   (setq _RMSE_p (float __RMSE_p))
   (setq _RMSE_v (float __RMSE_v))
   (setq _det_p (float __det_p))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:GT_pose
   (&rest __GT_pose)
   (if (keywordp (car __GT_pose))
       (send* _GT_pose __GT_pose)
     (progn
       (if __GT_pose (setq _GT_pose (car __GT_pose)))
       _GT_pose)))
  (:est_pose
   (&rest __est_pose)
   (if (keywordp (car __est_pose))
       (send* _est_pose __est_pose)
     (progn
       (if __est_pose (setq _est_pose (car __est_pose)))
       _est_pose)))
  (:GT_twist
   (&rest __GT_twist)
   (if (keywordp (car __GT_twist))
       (send* _GT_twist __GT_twist)
     (progn
       (if __GT_twist (setq _GT_twist (car __GT_twist)))
       _GT_twist)))
  (:est_twist
   (&rest __est_twist)
   (if (keywordp (car __est_twist))
       (send* _est_twist __est_twist)
     (progn
       (if __est_twist (setq _est_twist (car __est_twist)))
       _est_twist)))
  (:RMSE_p
   (&optional __RMSE_p)
   (if __RMSE_p (setq _RMSE_p __RMSE_p)) _RMSE_p)
  (:RMSE_v
   (&optional __RMSE_v)
   (if __RMSE_v (setq _RMSE_v __RMSE_v)) _RMSE_v)
  (:det_p
   (&optional __det_p)
   (if __det_p (setq _det_p __det_p)) _det_p)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _GT_pose
    (send _GT_pose :serialization-length)
    ;; geometry_msgs/Pose _est_pose
    (send _est_pose :serialization-length)
    ;; geometry_msgs/Twist _GT_twist
    (send _GT_twist :serialization-length)
    ;; geometry_msgs/Twist _est_twist
    (send _est_twist :serialization-length)
    ;; float64 _RMSE_p
    8
    ;; float64 _RMSE_v
    8
    ;; float64 _det_p
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _GT_pose
       (send _GT_pose :serialize s)
     ;; geometry_msgs/Pose _est_pose
       (send _est_pose :serialize s)
     ;; geometry_msgs/Twist _GT_twist
       (send _GT_twist :serialize s)
     ;; geometry_msgs/Twist _est_twist
       (send _est_twist :serialize s)
     ;; float64 _RMSE_p
       (sys::poke _RMSE_p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _RMSE_v
       (sys::poke _RMSE_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _det_p
       (sys::poke _det_p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _GT_pose
     (send _GT_pose :deserialize buf ptr-) (incf ptr- (send _GT_pose :serialization-length))
   ;; geometry_msgs/Pose _est_pose
     (send _est_pose :deserialize buf ptr-) (incf ptr- (send _est_pose :serialization-length))
   ;; geometry_msgs/Twist _GT_twist
     (send _GT_twist :deserialize buf ptr-) (incf ptr- (send _GT_twist :serialization-length))
   ;; geometry_msgs/Twist _est_twist
     (send _est_twist :deserialize buf ptr-) (incf ptr- (send _est_twist :serialization-length))
   ;; float64 _RMSE_p
     (setq _RMSE_p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _RMSE_v
     (setq _RMSE_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _det_p
     (setq _det_p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get state_estimation::Plot :md5sum-) "789b79485aceaa0b29456291c24a8393")
(setf (get state_estimation::Plot :datatype-) "state_estimation/Plot")
(setf (get state_estimation::Plot :definition-)
      "Header header
geometry_msgs/Pose GT_pose
geometry_msgs/Pose est_pose
geometry_msgs/Twist GT_twist
geometry_msgs/Twist est_twist
float64 RMSE_p
float64 RMSE_v
float64 det_p

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

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



(provide :state_estimation/Plot "789b79485aceaa0b29456291c24a8393")


