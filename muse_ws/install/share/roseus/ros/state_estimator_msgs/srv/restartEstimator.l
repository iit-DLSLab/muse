;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::restartEstimator)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'restartEstimator (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATOR")
  (make-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATOR"))
(unless (find-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATORREQUEST")
  (make-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATORREQUEST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATORRESPONSE")
  (make-package "STATE_ESTIMATOR_MSGS::RESTARTESTIMATORRESPONSE"))

(in-package "ROS")





(defclass state_estimator_msgs::restartEstimatorRequest
  :super ros::object
  :slots (_name ))

(defmethod state_estimator_msgs::restartEstimatorRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass state_estimator_msgs::restartEstimatorResponse
  :super ros::object
  :slots (_success ))

(defmethod state_estimator_msgs::restartEstimatorResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass state_estimator_msgs::restartEstimator
  :super ros::object
  :slots ())

(setf (get state_estimator_msgs::restartEstimator :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get state_estimator_msgs::restartEstimator :datatype-) "state_estimator_msgs/restartEstimator")
(setf (get state_estimator_msgs::restartEstimator :request) state_estimator_msgs::restartEstimatorRequest)
(setf (get state_estimator_msgs::restartEstimator :response) state_estimator_msgs::restartEstimatorResponse)

(defmethod state_estimator_msgs::restartEstimatorRequest
  (:response () (instance state_estimator_msgs::restartEstimatorResponse :init)))

(setf (get state_estimator_msgs::restartEstimatorRequest :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get state_estimator_msgs::restartEstimatorRequest :datatype-) "state_estimator_msgs/restartEstimatorRequest")
(setf (get state_estimator_msgs::restartEstimatorRequest :definition-)
      "string name
---
bool success

")

(setf (get state_estimator_msgs::restartEstimatorResponse :md5sum-) "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(setf (get state_estimator_msgs::restartEstimatorResponse :datatype-) "state_estimator_msgs/restartEstimatorResponse")
(setf (get state_estimator_msgs::restartEstimatorResponse :definition-)
      "string name
---
bool success

")



(provide :state_estimator_msgs/restartEstimator "d08a3b641c2f8680fbdfb1ea2e17a3e1")


