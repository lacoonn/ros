;; Auto-generated. Do not edit!


(when (boundp 'knu_ros_lecture::srvKnuRosLecture)
  (if (not (find-package "KNU_ROS_LECTURE"))
    (make-package "KNU_ROS_LECTURE"))
  (shadow 'srvKnuRosLecture (find-package "KNU_ROS_LECTURE")))
(unless (find-package "KNU_ROS_LECTURE::SRVKNUROSLECTURE")
  (make-package "KNU_ROS_LECTURE::SRVKNUROSLECTURE"))
(unless (find-package "KNU_ROS_LECTURE::SRVKNUROSLECTUREREQUEST")
  (make-package "KNU_ROS_LECTURE::SRVKNUROSLECTUREREQUEST"))
(unless (find-package "KNU_ROS_LECTURE::SRVKNUROSLECTURERESPONSE")
  (make-package "KNU_ROS_LECTURE::SRVKNUROSLECTURERESPONSE"))

(in-package "ROS")





(defclass knu_ros_lecture::srvKnuRosLectureRequest
  :super ros::object
  :slots (_a _b ))

(defmethod knu_ros_lecture::srvKnuRosLectureRequest
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int64 _a
    8
    ;; int64 _b
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;; int64 _b
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _b (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _b) (= (length (_b . bv)) 2)) ;; bignum
              (write-long (ash (elt (_b . bv) 0) 0) s)
              (write-long (ash (elt (_b . bv) 1) -1) s))
             ((and (class _b) (= (length (_b . bv)) 1)) ;; big1
              (write-long (elt (_b . bv) 0) s)
              (write-long (if (>= _b 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _b s)(write-long (if (>= _b 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _b
#+(or :alpha :irix6 :x86_64)
      (setf _b (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _b (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass knu_ros_lecture::srvKnuRosLectureResponse
  :super ros::object
  :slots (_result ))

(defmethod knu_ros_lecture::srvKnuRosLectureResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int64 _result
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _result
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _result (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _result) (= (length (_result . bv)) 2)) ;; bignum
              (write-long (ash (elt (_result . bv) 0) 0) s)
              (write-long (ash (elt (_result . bv) 1) -1) s))
             ((and (class _result) (= (length (_result . bv)) 1)) ;; big1
              (write-long (elt (_result . bv) 0) s)
              (write-long (if (>= _result 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _result s)(write-long (if (>= _result 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _result
#+(or :alpha :irix6 :x86_64)
      (setf _result (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _result (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass knu_ros_lecture::srvKnuRosLecture
  :super ros::object
  :slots ())

(setf (get knu_ros_lecture::srvKnuRosLecture :md5sum-) "d431cae597499d244ef1c576e21358c8")
(setf (get knu_ros_lecture::srvKnuRosLecture :datatype-) "knu_ros_lecture/srvKnuRosLecture")
(setf (get knu_ros_lecture::srvKnuRosLecture :request) knu_ros_lecture::srvKnuRosLectureRequest)
(setf (get knu_ros_lecture::srvKnuRosLecture :response) knu_ros_lecture::srvKnuRosLectureResponse)

(defmethod knu_ros_lecture::srvKnuRosLectureRequest
  (:response () (instance knu_ros_lecture::srvKnuRosLectureResponse :init)))

(setf (get knu_ros_lecture::srvKnuRosLectureRequest :md5sum-) "d431cae597499d244ef1c576e21358c8")
(setf (get knu_ros_lecture::srvKnuRosLectureRequest :datatype-) "knu_ros_lecture/srvKnuRosLectureRequest")
(setf (get knu_ros_lecture::srvKnuRosLectureRequest :definition-)
      "int64 a
int64 b
---
int64 result

")

(setf (get knu_ros_lecture::srvKnuRosLectureResponse :md5sum-) "d431cae597499d244ef1c576e21358c8")
(setf (get knu_ros_lecture::srvKnuRosLectureResponse :datatype-) "knu_ros_lecture/srvKnuRosLectureResponse")
(setf (get knu_ros_lecture::srvKnuRosLectureResponse :definition-)
      "int64 a
int64 b
---
int64 result

")



(provide :knu_ros_lecture/srvKnuRosLecture "d431cae597499d244ef1c576e21358c8")


