
(cl:in-package :asdf)

(defsystem "state_estimator_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ContactDetection" :depends-on ("_package_ContactDetection"))
    (:file "_package_ContactDetection" :depends-on ("_package"))
    (:file "JointStateWithAcceleration" :depends-on ("_package_JointStateWithAcceleration"))
    (:file "_package_JointStateWithAcceleration" :depends-on ("_package"))
    (:file "LegOdometry" :depends-on ("_package_LegOdometry"))
    (:file "_package_LegOdometry" :depends-on ("_package"))
    (:file "attitude" :depends-on ("_package_attitude"))
    (:file "_package_attitude" :depends-on ("_package"))
    (:file "sensor_fusion" :depends-on ("_package_sensor_fusion"))
    (:file "_package_sensor_fusion" :depends-on ("_package"))
  ))