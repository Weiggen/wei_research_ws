;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::WeightArray)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'WeightArray (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::WEIGHTARRAY")
  (make-package "VORONOI_CBSA::WEIGHTARRAY"))

(in-package "ROS")
;;//! \htmlinclude WeightArray.msg.html


(defclass voronoi_cbsa::WeightArray
  :super ros::object
  :slots (_weights ))

(defmethod voronoi_cbsa::WeightArray
  (:init
   (&key
    ((:weights __weights) ())
    )
   (send-super :init)
   (setq _weights __weights)
   self)
  (:weights
   (&rest __weights)
   (if (keywordp (car __weights))
       (send* _weights __weights)
     (progn
       (if __weights (setq _weights (car __weights)))
       _weights)))
  (:serialization-length
   ()
   (+
    ;; voronoi_cbsa/Weight[] _weights
    (apply #'+ (send-all _weights :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; voronoi_cbsa/Weight[] _weights
     (write-long (length _weights) s)
     (dolist (elem _weights)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; voronoi_cbsa/Weight[] _weights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _weights (let (r) (dotimes (i n) (push (instance voronoi_cbsa::Weight :init) r)) r))
     (dolist (elem- _weights)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get voronoi_cbsa::WeightArray :md5sum-) "a52d516a208ee351d816a3ad44a096ad")
(setf (get voronoi_cbsa::WeightArray :datatype-) "voronoi_cbsa/WeightArray")
(setf (get voronoi_cbsa::WeightArray :definition-)
      "Weight[] weights
================================================================================
MSG: voronoi_cbsa/Weight
string  type
int16   event_id
float64 score
")



(provide :voronoi_cbsa/WeightArray "a52d516a208ee351d816a3ad44a096ad")


