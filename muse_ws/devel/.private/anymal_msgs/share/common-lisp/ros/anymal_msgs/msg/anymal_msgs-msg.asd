
(cl:in-package :asdf)

(defsystem "anymal_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnymalState" :depends-on ("_package_AnymalState"))
    (:file "_package_AnymalState" :depends-on ("_package"))
    (:file "Contact" :depends-on ("_package_Contact"))
    (:file "_package_Contact" :depends-on ("_package"))
    (:file "ExtendedJointState" :depends-on ("_package_ExtendedJointState"))
    (:file "_package_ExtendedJointState" :depends-on ("_package"))
  ))