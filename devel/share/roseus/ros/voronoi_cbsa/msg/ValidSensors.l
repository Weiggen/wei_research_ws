;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::ValidSensors)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'ValidSensors (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::VALIDSENSORS")
  (make-package "VORONOI_CBSA::VALIDSENSORS"))

(in-package "ROS")
;;//! \htmlinclude ValidSensors.msg.html


(defclass voronoi_cbsa::ValidSensors
  :super ros::object
  :slots (_data ))

(defmethod voronoi_cbsa::ValidSensors
  (:init
   (&key
    ((:data __data) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string[] _data
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _data i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::ValidSensors :md5sum-) "cce5a364f3a3be12c9722c6dcad2fa94")
(setf (get voronoi_cbsa::ValidSensors :datatype-) "voronoi_cbsa/ValidSensors")
(setf (get voronoi_cbsa::ValidSensors :definition-)
      "string[] data

")



(provide :voronoi_cbsa/ValidSensors "cce5a364f3a3be12c9722c6dcad2fa94")


