; Auto-generated. Do not edit!


(cl:in-package knu_ros_lecture-msg)


;//! \htmlinclude knuRosLecture.msg.html

(cl:defclass <knuRosLecture> (roslisp-msg-protocol:ros-message)
  ((int32Data
    :reader int32Data
    :initarg :int32Data
    :type cl:integer
    :initform 0)
   (float32Data
    :reader float32Data
    :initarg :float32Data
    :type cl:float
    :initform 0.0)
   (float64Data
    :reader float64Data
    :initarg :float64Data
    :type cl:float
    :initform 0.0)
   (stringData
    :reader stringData
    :initarg :stringData
    :type cl:string
    :initform ""))
)

(cl:defclass knuRosLecture (<knuRosLecture>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <knuRosLecture>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'knuRosLecture)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name knu_ros_lecture-msg:<knuRosLecture> is deprecated: use knu_ros_lecture-msg:knuRosLecture instead.")))

(cl:ensure-generic-function 'int32Data-val :lambda-list '(m))
(cl:defmethod int32Data-val ((m <knuRosLecture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-msg:int32Data-val is deprecated.  Use knu_ros_lecture-msg:int32Data instead.")
  (int32Data m))

(cl:ensure-generic-function 'float32Data-val :lambda-list '(m))
(cl:defmethod float32Data-val ((m <knuRosLecture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-msg:float32Data-val is deprecated.  Use knu_ros_lecture-msg:float32Data instead.")
  (float32Data m))

(cl:ensure-generic-function 'float64Data-val :lambda-list '(m))
(cl:defmethod float64Data-val ((m <knuRosLecture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-msg:float64Data-val is deprecated.  Use knu_ros_lecture-msg:float64Data instead.")
  (float64Data m))

(cl:ensure-generic-function 'stringData-val :lambda-list '(m))
(cl:defmethod stringData-val ((m <knuRosLecture>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader knu_ros_lecture-msg:stringData-val is deprecated.  Use knu_ros_lecture-msg:stringData instead.")
  (stringData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <knuRosLecture>) ostream)
  "Serializes a message object of type '<knuRosLecture>"
  (cl:let* ((signed (cl:slot-value msg 'int32Data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'float32Data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'float64Data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'stringData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'stringData))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <knuRosLecture>) istream)
  "Deserializes a message object of type '<knuRosLecture>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'int32Data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'float32Data) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'float64Data) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stringData) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'stringData) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<knuRosLecture>)))
  "Returns string type for a message object of type '<knuRosLecture>"
  "knu_ros_lecture/knuRosLecture")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'knuRosLecture)))
  "Returns string type for a message object of type 'knuRosLecture"
  "knu_ros_lecture/knuRosLecture")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<knuRosLecture>)))
  "Returns md5sum for a message object of type '<knuRosLecture>"
  "a47b7b41e31bbd789a10dc658a18ec37")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'knuRosLecture)))
  "Returns md5sum for a message object of type 'knuRosLecture"
  "a47b7b41e31bbd789a10dc658a18ec37")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<knuRosLecture>)))
  "Returns full string definition for message of type '<knuRosLecture>"
  (cl:format cl:nil "int32 int32Data~%float32 float32Data~%float64 float64Data~%string stringData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'knuRosLecture)))
  "Returns full string definition for message of type 'knuRosLecture"
  (cl:format cl:nil "int32 int32Data~%float32 float32Data~%float64 float64Data~%string stringData~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <knuRosLecture>))
  (cl:+ 0
     4
     4
     8
     4 (cl:length (cl:slot-value msg 'stringData))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <knuRosLecture>))
  "Converts a ROS message object to a list"
  (cl:list 'knuRosLecture
    (cl:cons ':int32Data (int32Data msg))
    (cl:cons ':float32Data (float32Data msg))
    (cl:cons ':float64Data (float64Data msg))
    (cl:cons ':stringData (stringData msg))
))
