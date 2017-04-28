;; Auto-generated. Do not edit!


(when (boundp 'knu_ros_lecture::knuRosLecture)
  (if (not (find-package "KNU_ROS_LECTURE"))
    (make-package "KNU_ROS_LECTURE"))
  (shadow 'knuRosLecture (find-package "KNU_ROS_LECTURE")))
(unless (find-package "KNU_ROS_LECTURE::KNUROSLECTURE")
  (make-package "KNU_ROS_LECTURE::KNUROSLECTURE"))

(in-package "ROS")
;;//! \htmlinclude knuRosLecture.msg.html


(defclass knu_ros_lecture::knuRosLecture
  :super ros::object
  :slots (_int32Data _float32Data _float64Data _stringData ))

(defmethod knu_ros_lecture::knuRosLecture
  (:init
   (&key
    ((:int32Data __int32Data) 0)
    ((:float32Data __float32Data) 0.0)
    ((:float64Data __float64Data) 0.0)
    ((:stringData __stringData) "")
    )
   (send-super :init)
   (setq _int32Data (round __int32Data))
   (setq _float32Data (float __float32Data))
   (setq _float64Data (float __float64Data))
   (setq _stringData (string __stringData))
   self)
  (:int32Data
   (&optional __int32Data)
   (if __int32Data (setq _int32Data __int32Data)) _int32Data)
  (:float32Data
   (&optional __float32Data)
   (if __float32Data (setq _float32Data __float32Data)) _float32Data)
  (:float64Data
   (&optional __float64Data)
   (if __float64Data (setq _float64Data __float64Data)) _float64Data)
  (:stringData
   (&optional __stringData)
   (if __stringData (setq _stringData __stringData)) _stringData)
  (:serialization-length
   ()
   (+
    ;; int32 _int32Data
    4
    ;; float32 _float32Data
    4
    ;; float64 _float64Data
    8
    ;; string _stringData
    4 (length _stringData)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _int32Data
       (write-long _int32Data s)
     ;; float32 _float32Data
       (sys::poke _float32Data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _float64Data
       (sys::poke _float64Data (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _stringData
       (write-long (length _stringData) s) (princ _stringData s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _int32Data
     (setq _int32Data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _float32Data
     (setq _float32Data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _float64Data
     (setq _float64Data (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _stringData
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _stringData (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get knu_ros_lecture::knuRosLecture :md5sum-) "a47b7b41e31bbd789a10dc658a18ec37")
(setf (get knu_ros_lecture::knuRosLecture :datatype-) "knu_ros_lecture/knuRosLecture")
(setf (get knu_ros_lecture::knuRosLecture :definition-)
      "int32 int32Data
float32 float32Data
float64 float64Data
string stringData

")



(provide :knu_ros_lecture/knuRosLecture "a47b7b41e31bbd789a10dc658a18ec37")


