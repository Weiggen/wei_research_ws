; Auto-generated. Do not edit!


(cl:in-package voronoi_cbsa-msg)


;//! \htmlinclude VoteList.msg.html

(cl:defclass <VoteList> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (vote
    :reader vote
    :initarg :vote
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VoteList (<VoteList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoteList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoteList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_cbsa-msg:<VoteList> is deprecated: use voronoi_cbsa-msg:VoteList instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <VoteList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:index-val is deprecated.  Use voronoi_cbsa-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'vote-val :lambda-list '(m))
(cl:defmethod vote-val ((m <VoteList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_cbsa-msg:vote-val is deprecated.  Use voronoi_cbsa-msg:vote instead.")
  (vote m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoteList>) ostream)
  "Serializes a message object of type '<VoteList>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vote) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoteList>) istream)
  "Deserializes a message object of type '<VoteList>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'vote) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoteList>)))
  "Returns string type for a message object of type '<VoteList>"
  "voronoi_cbsa/VoteList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoteList)))
  "Returns string type for a message object of type 'VoteList"
  "voronoi_cbsa/VoteList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoteList>)))
  "Returns md5sum for a message object of type '<VoteList>"
  "4974848ca173ad3d0044de0990068ffc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoteList)))
  "Returns md5sum for a message object of type 'VoteList"
  "4974848ca173ad3d0044de0990068ffc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoteList>)))
  "Returns full string definition for message of type '<VoteList>"
  (cl:format cl:nil "int64 index~%bool vote~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoteList)))
  "Returns full string definition for message of type 'VoteList"
  (cl:format cl:nil "int64 index~%bool vote~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoteList>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoteList>))
  "Converts a ROS message object to a list"
  (cl:list 'VoteList
    (cl:cons ':index (index msg))
    (cl:cons ':vote (vote msg))
))
