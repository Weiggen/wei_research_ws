;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::Sensor)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'Sensor (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::SENSOR")
  (make-package "VORONOI_CBSA::SENSOR"))

(in-package "ROS")
;;//! \htmlinclude Sensor.msg.html


(defclass voronoi_cbsa::Sensor
  :super ros::object
  :slots (_type _score ))

(defmethod voronoi_cbsa::Sensor
  (:init
   (&key
    ((:type __type) "")
    ((:score __score) 0.0)
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _score (float __score))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; float64 _score
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get voronoi_cbsa::Sensor :md5sum-) "ac5a2170d45f896240e0e9487f638f4a")
(setf (get voronoi_cbsa::Sensor :datatype-) "voronoi_cbsa/Sensor")
(setf (get voronoi_cbsa::Sensor :definition-)
      "string type
float64 score
")



(provide :voronoi_cbsa/Sensor "ac5a2170d45f896240e0e9487f638f4a")


