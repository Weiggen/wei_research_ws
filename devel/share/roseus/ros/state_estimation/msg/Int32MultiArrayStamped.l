;; Auto-generated. Do not edit!


(when (boundp 'state_estimation::Int32MultiArrayStamped)
  (if (not (find-package "STATE_ESTIMATION"))
    (make-package "STATE_ESTIMATION"))
  (shadow 'Int32MultiArrayStamped (find-package "STATE_ESTIMATION")))
(unless (find-package "STATE_ESTIMATION::INT32MULTIARRAYSTAMPED")
  (make-package "STATE_ESTIMATION::INT32MULTIARRAYSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude Int32MultiArrayStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimation::Int32MultiArrayStamped
  :super ros::object
  :slots (_header _data ))

(defmethod state_estimation::Int32MultiArrayStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _data
    (* 4    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (write-long (elt _data i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get state_estimation::Int32MultiArrayStamped :md5sum-) "1ce4762ce13f3d9e1f17586acc253067")
(setf (get state_estimation::Int32MultiArrayStamped :datatype-) "state_estimation/Int32MultiArrayStamped")
(setf (get state_estimation::Int32MultiArrayStamped :definition-)
      "Header header
int32[] data
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



(provide :state_estimation/Int32MultiArrayStamped "1ce4762ce13f3d9e1f17586acc253067")


