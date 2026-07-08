;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::getWhitelist)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'getWhitelist (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETWHITELIST")
  (make-package "STATE_ESTIMATOR_MSGS::GETWHITELIST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETWHITELISTREQUEST")
  (make-package "STATE_ESTIMATOR_MSGS::GETWHITELISTREQUEST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETWHITELISTRESPONSE")
  (make-package "STATE_ESTIMATOR_MSGS::GETWHITELISTRESPONSE"))

(in-package "ROS")





(defclass state_estimator_msgs::getWhitelistRequest
  :super ros::object
  :slots ())

(defmethod state_estimator_msgs::getWhitelistRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass state_estimator_msgs::getWhitelistResponse
  :super ros::object
  :slots (_names ))

(defmethod state_estimator_msgs::getWhitelistResponse
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _names __names)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass state_estimator_msgs::getWhitelist
  :super ros::object
  :slots ())

(setf (get state_estimator_msgs::getWhitelist :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getWhitelist :datatype-) "state_estimator_msgs/getWhitelist")
(setf (get state_estimator_msgs::getWhitelist :request) state_estimator_msgs::getWhitelistRequest)
(setf (get state_estimator_msgs::getWhitelist :response) state_estimator_msgs::getWhitelistResponse)

(defmethod state_estimator_msgs::getWhitelistRequest
  (:response () (instance state_estimator_msgs::getWhitelistResponse :init)))

(setf (get state_estimator_msgs::getWhitelistRequest :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getWhitelistRequest :datatype-) "state_estimator_msgs/getWhitelistRequest")
(setf (get state_estimator_msgs::getWhitelistRequest :definition-)
      "---
string[] names

")

(setf (get state_estimator_msgs::getWhitelistResponse :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getWhitelistResponse :datatype-) "state_estimator_msgs/getWhitelistResponse")
(setf (get state_estimator_msgs::getWhitelistResponse :definition-)
      "---
string[] names

")



(provide :state_estimator_msgs/getWhitelist "dc7ae3609524b18034e49294a4ce670e")


