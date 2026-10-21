
(cl:in-package :asdf)

(defsystem "imu_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "InsState15" :depends-on ("_package_InsState15"))
    (:file "_package_InsState15" :depends-on ("_package"))
  ))