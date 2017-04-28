; Auto-generated. Do not edit!


(cl:in-package knu_ros_lecture-srv)


;//! \htmlinclude srvKnuRosLecture-request.msg.html

(cl:defclass <srvKnuRosLecture-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass srvKnuRosLecture-request (<srvKnuRosLecture-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvKnuRosLecture-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvKnuRosLecture-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name knu_ros_lecture-srv:<srvKnuRosLecture-request> is deprecated: use knu_ros_lecture-srv:srvKnuRosLecture-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <srvKnuRosLecture-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-srv:a-val is deprecated.  Use knu_ros_lecture-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <srvKnuRosLecture-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-srv:b-val is deprecated.  Use knu_ros_lecture-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvKnuRosLecture-request>) ostream)
  "Serializes a message object of type '<srvKnuRosLecture-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvKnuRosLecture-request>) istream)
  "Deserializes a message object of type '<srvKnuRosLecture-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvKnuRosLecture-request>)))
  "Returns string type for a service object of type '<srvKnuRosLecture-request>"
  "knu_ros_lecture/srvKnuRosLectureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvKnuRosLecture-request)))
  "Returns string type for a service object of type 'srvKnuRosLecture-request"
  "knu_ros_lecture/srvKnuRosLectureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvKnuRosLecture-request>)))
  "Returns md5sum for a message object of type '<srvKnuRosLecture-request>"
  "d431cae597499d244ef1c576e21358c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvKnuRosLecture-request)))
  "Returns md5sum for a message object of type 'srvKnuRosLecture-request"
  "d431cae597499d244ef1c576e21358c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvKnuRosLecture-request>)))
  "Returns full string definition for message of type '<srvKnuRosLecture-request>"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvKnuRosLecture-request)))
  "Returns full string definition for message of type 'srvKnuRosLecture-request"
  (cl:format cl:nil "int64 a~%int64 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvKnuRosLecture-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvKnuRosLecture-request>))
  "Converts a ROS message object to a list"
  (cl:list 'srvKnuRosLecture-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude srvKnuRosLecture-response.msg.html

(cl:defclass <srvKnuRosLecture-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass srvKnuRosLecture-response (<srvKnuRosLecture-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <srvKnuRosLecture-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'srvKnuRosLecture-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name knu_ros_lecture-srv:<srvKnuRosLecture-response> is deprecated: use knu_ros_lecture-srv:srvKnuRosLecture-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <srvKnuRosLecture-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-srv:result-val is deprecated.  Use knu_ros_lecture-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <srvKnuRosLecture-response>) ostream)
  "Serializes a message object of type '<srvKnuRosLecture-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <srvKnuRosLecture-response>) istream)
  "Deserializes a message object of type '<srvKnuRosLecture-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<srvKnuRosLecture-response>)))
  "Returns string type for a service object of type '<srvKnuRosLecture-response>"
  "knu_ros_lecture/srvKnuRosLectureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvKnuRosLecture-response)))
  "Returns string type for a service object of type 'srvKnuRosLecture-response"
  "knu_ros_lecture/srvKnuRosLectureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<srvKnuRosLecture-response>)))
  "Returns md5sum for a message object of type '<srvKnuRosLecture-response>"
  "d431cae597499d244ef1c576e21358c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'srvKnuRosLecture-response)))
  "Returns md5sum for a message object of type 'srvKnuRosLecture-response"
  "d431cae597499d244ef1c576e21358c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<srvKnuRosLecture-response>)))
  "Returns full string definition for message of type '<srvKnuRosLecture-response>"
  (cl:format cl:nil "int64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'srvKnuRosLecture-response)))
  "Returns full string definition for message of type 'srvKnuRosLecture-response"
  (cl:format cl:nil "int64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <srvKnuRosLecture-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <srvKnuRosLecture-response>))
  "Converts a ROS message object to a list"
  (cl:list 'srvKnuRosLecture-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'srvKnuRosLecture)))
  'srvKnuRosLecture-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'srvKnuRosLecture)))
  'srvKnuRosLecture-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'srvKnuRosLecture)))
  "Returns string type for a service object of type '<srvKnuRosLecture>"
  "knu_ros_lecture/srvKnuRosLecture")