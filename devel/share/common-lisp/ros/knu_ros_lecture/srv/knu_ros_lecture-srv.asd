
(cl:in-package :asdf)

(defsystem "knu_ros_lecture-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srvKnuRosLecture" :depends-on ("_package_srvKnuRosLecture"))
    (:file "_package_srvKnuRosLecture" :depends-on ("_package"))
  ))