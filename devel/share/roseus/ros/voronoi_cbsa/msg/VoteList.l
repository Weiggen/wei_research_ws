;; Auto-generated. Do not edit!


(when (boundp 'voronoi_cbsa::VoteList)
  (if (not (find-package "VORONOI_CBSA"))
    (make-package "VORONOI_CBSA"))
  (shadow 'VoteList (find-package "VORONOI_CBSA")))
(unless (find-package "VORONOI_CBSA::VOTELIST")
  (make-package "VORONOI_CBSA::VOTELIST"))

(in-package "ROS")
;;//! \htmlinclude VoteList.msg.html


(defclass voronoi_cbsa::VoteList
  :super ros::object
  :slots (_index _vote ))

(defmethod voronoi_cbsa::VoteList
  (:init
   (&key
    ((:index __index) 0)
    ((:vote __vote) nil)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _vote __vote)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:vote
   (&optional (__vote :null))
   (if (not (eq __vote :null)) (setq _vote __vote)) _vote)
  (:serialization-length
   ()
   (+
    ;; int64 _index
    8
    ;; bool _vote
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _index
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _index (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _index) (= (length (_index . bv)) 2)) ;; bignum
              (write-long (ash (elt (_index . bv) 0) 0) s)
              (write-long (ash (elt (_index . bv) 1) -1) s))
             ((and (class _index) (= (length (_index . bv)) 1)) ;; big1
              (write-long (elt (_index . bv) 0) s)
              (write-long (if (>= _index 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _index s)(write-long (if (>= _index 0) 0 #xffffffff) s)))
     ;; bool _vote
       (if _vote (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _index
#+(or :alpha :irix6 :x86_64)
      (setf _index (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _index (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _vote
     (setq _vote (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get voronoi_cbsa::VoteList :md5sum-) "4974848ca173ad3d0044de0990068ffc")
(setf (get voronoi_cbsa::VoteList :datatype-) "voronoi_cbsa/VoteList")
(setf (get voronoi_cbsa::VoteList :definition-)
      "int64 index
bool vote

")



(provide :voronoi_cbsa/VoteList "4974848ca173ad3d0044de0990068ffc")


