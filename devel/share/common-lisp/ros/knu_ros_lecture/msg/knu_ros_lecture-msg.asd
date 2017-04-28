
(cl:in-package :asdf)

(defsystem "knu_ros_lecture-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "knuRosLecture" :depends-on ("_package_knuRosLecture"))
    (:file "_package_knuRosLecture" :depends-on ("_package"))
  ))