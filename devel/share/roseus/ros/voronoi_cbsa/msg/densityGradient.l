;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::densityGradient)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'densityGradient (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::DENSITYGRADIENT")
  (make-package "VORONOI_CBSA::DENSITYGRADIENT"))

(in-package "ROS")
;;//! \htmlinclude densityGradient.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass voronoi_cbsa::densityGradient
  :super ros::object
  :slots (_header _gradient_x _gradient_y ))

(defmethod voronoi_cbsa::densityGradient
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:gradient_x __gradient_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:gradient_y __gradient_y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _gradient_x __gradient_x)
   (setq _gradient_y __gradient_y)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:gradient_x
   (&optional __gradient_x)
   (if __gradient_x (setq _gradient_x __gradient_x)) _gradient_x)
  (:gradient_y
   (&optional __gradient_y)
   (if __gradient_y (setq _gradient_y __gradient_y)) _gradient_y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _gradient_x
    (* 8    (length _gradient_x)) 4
    ;; float64[] _gradient_y
    (* 8    (length _gradient_y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _gradient_x
     (write-long (length _gradient_x) s)
     (dotimes (i (length _gradient_x))
       (sys::poke (elt _gradient_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _gradient_y
     (write-long (length _gradient_y) s)
     (dotimes (i (length _gradient_y))
       (sys::poke (elt _gradient_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _gradient_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gradient_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gradient_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _gradient_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gradient_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gradient_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::densityGradient :md5sum-) "cbfc8681c266124a45b98ba27e25a5f7")
(setf (get voronoi_cbsa::densityGradient :datatype-) "voronoi_cbsa/densityGradient")
(setf (get voronoi_cbsa::densityGradient :definition-)
      "Header header
float64[] gradient_x
float64[] gradient_y
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

")



(provide :voronoi_cbsa/densityGradient "cbfc8681c266124a45b98ba27e25a5f7")


