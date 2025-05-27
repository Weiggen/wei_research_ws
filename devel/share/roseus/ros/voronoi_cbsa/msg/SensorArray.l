;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::SensorArray)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'SensorArray (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::SENSORARRAY")
  (make-package "VORONOI_CBSA::SENSORARRAY"))

(in-package "ROS")
;;//! \htmlinclude SensorArray.msg.html


(defclass voronoi_cbsa::SensorArray
  :super ros::object
  :slots (_sensors ))

(defmethod voronoi_cbsa::SensorArray
  (:init
   (&key
    ((:sensors __sensors) ())
    )
   (send-super :init)
   (setq _sensors __sensors)
   self)
  (:sensors
   (&rest __sensors)
   (if (keywordp (car __sensors))
       (send* _sensors __sensors)
     (progn
       (if __sensors (setq _sensors (car __sensors)))
       _sensors)))
  (:serialization-length
   ()
   (+
    ;; voronoi_cbsa/Sensor[] _sensors
    (apply #'+ (send-all _sensors :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; voronoi_cbsa/Sensor[] _sensors
     (write-long (length _sensors) s)
     (dolist (elem _sensors)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; voronoi_cbsa/Sensor[] _sensors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensors (let (r) (dotimes (i n) (push (instance voronoi_cbsa::Sensor :init) r)) r))
     (dolist (elem- _sensors)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::SensorArray :md5sum-) "e1d908c1ca577e30068931de829be081")
(setf (get voronoi_cbsa::SensorArray :datatype-) "voronoi_cbsa/SensorArray")
(setf (get voronoi_cbsa::SensorArray :definition-)
      "Sensor[] sensors
================================================================================
MSG: voronoi_cbsa/Sensor
string type
float64 score
")



(provide :voronoi_cbsa/SensorArray "e1d908c1ca577e30068931de829be081")


