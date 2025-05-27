;; Auto-generated. Do not edit!


(when (boundp 'state_estimation::EIFpairStamped)
  (if (not (find-package "STATE_ESTIMATION"))
    (make-package "STATE_ESTIMATION"))
  (shadow 'EIFpairStamped (find-package "STATE_ESTIMATION")))
(unless (find-package "STATE_ESTIMATION::EIFPAIRSTAMPED")
  (make-package "STATE_ESTIMATION::EIFPAIRSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude EIFpairStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimation::EIFpairStamped
  :super ros::object
  :slots (_header _stateSize _id _P_hat _X_hat _s _y ))

(defmethod state_estimation::EIFpairStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stateSize __stateSize) 0)
    ((:id __id) 0)
    ((:P_hat __P_hat) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:X_hat __X_hat) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:s __s) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _stateSize (round __stateSize))
   (setq _id (round __id))
   (setq _P_hat __P_hat)
   (setq _X_hat __X_hat)
   (setq _s __s)
   (setq _y __y)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:stateSize
   (&optional __stateSize)
   (if __stateSize (setq _stateSize __stateSize)) _stateSize)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:P_hat
   (&optional __P_hat)
   (if __P_hat (setq _P_hat __P_hat)) _P_hat)
  (:X_hat
   (&optional __X_hat)
   (if __X_hat (setq _X_hat __X_hat)) _X_hat)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _stateSize
    4
    ;; int32 _id
    4
    ;; float64[] _P_hat
    (* 8    (length _P_hat)) 4
    ;; float64[] _X_hat
    (* 8    (length _X_hat)) 4
    ;; float64[] _s
    (* 8    (length _s)) 4
    ;; float64[] _y
    (* 8    (length _y)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _stateSize
       (write-long _stateSize s)
     ;; int32 _id
       (write-long _id s)
     ;; float64[] _P_hat
     (write-long (length _P_hat) s)
     (dotimes (i (length _P_hat))
       (sys::poke (elt _P_hat i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _X_hat
     (write-long (length _X_hat) s)
     (dotimes (i (length _X_hat))
       (sys::poke (elt _X_hat i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _s
     (write-long (length _s) s)
     (dotimes (i (length _s))
       (sys::poke (elt _s i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _stateSize
     (setq _stateSize (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _P_hat
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _P_hat (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _P_hat i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _X_hat
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _X_hat (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _X_hat i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _s
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _s (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _s i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get state_estimation::EIFpairStamped :md5sum-) "26b414ae046293c626e824119402b4fd")
(setf (get state_estimation::EIFpairStamped :datatype-) "state_estimation/EIFpairStamped")
(setf (get state_estimation::EIFpairStamped :definition-)
      "Header header
int32 stateSize
int32 id
float64[] P_hat
float64[] X_hat
float64[] s
float64[] y


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



(provide :state_estimation/EIFpairStamped "26b414ae046293c626e824119402b4fd")


