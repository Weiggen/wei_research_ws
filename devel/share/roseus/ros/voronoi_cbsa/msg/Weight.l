;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::Weight)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'Weight (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::WEIGHT")
  (make-package "VORONOI_CBSA::WEIGHT"))

(in-package "ROS")
;;//! \htmlinclude Weight.msg.html


(defclass voronoi_cbsa::Weight
  :super ros::object
  :slots (_type _event_id _score ))

(defmethod voronoi_cbsa::Weight
  (:init
   (&key
    ((:type __type) "")
    ((:event_id __event_id) 0)
    ((:score __score) 0.0)
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _event_id (round __event_id))
   (setq _score (float __score))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:event_id
   (&optional __event_id)
   (if __event_id (setq _event_id __event_id)) _event_id)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; int16 _event_id
    2
    ;; float64 _score
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; int16 _event_id
       (write-word _event_id s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _event_id
     (setq _event_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get voronoi_cbsa::Weight :md5sum-) "114b2405c586c01c700456b3e7f87d9e")
(setf (get voronoi_cbsa::Weight :datatype-) "voronoi_cbsa/Weight")
(setf (get voronoi_cbsa::Weight :definition-)
      "string  type
int16   event_id
float64 score
")



(provide :voronoi_cbsa/Weight "114b2405c586c01c700456b3e7f87d9e")


