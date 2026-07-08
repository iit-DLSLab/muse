;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::getEstimatorDescription)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'getEstimatorDescription (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTION")
  (make-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTION"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTIONREQUEST")
  (make-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTIONREQUEST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTIONRESPONSE")
  (make-package "STATE_ESTIMATOR_MSGS::GETESTIMATORDESCRIPTIONRESPONSE"))

(in-package "ROS")





(defclass state_estimator_msgs::getEstimatorDescriptionRequest
  :super ros::object
  :slots (_name ))

(defmethod state_estimator_msgs::getEstimatorDescriptionRequest
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

(defclass state_estimator_msgs::getEstimatorDescriptionResponse
  :super ros::object
  :slots (_success _description ))

(defmethod state_estimator_msgs::getEstimatorDescriptionResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _description (string __description))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass state_estimator_msgs::getEstimatorDescription
  :super ros::object
  :slots ())

(setf (get state_estimator_msgs::getEstimatorDescription :md5sum-) "12c3019909e7fa7b3482cdf9594556c2")
(setf (get state_estimator_msgs::getEstimatorDescription :datatype-) "state_estimator_msgs/getEstimatorDescription")
(setf (get state_estimator_msgs::getEstimatorDescription :request) state_estimator_msgs::getEstimatorDescriptionRequest)
(setf (get state_estimator_msgs::getEstimatorDescription :response) state_estimator_msgs::getEstimatorDescriptionResponse)

(defmethod state_estimator_msgs::getEstimatorDescriptionRequest
  (:response () (instance state_estimator_msgs::getEstimatorDescriptionResponse :init)))

(setf (get state_estimator_msgs::getEstimatorDescriptionRequest :md5sum-) "12c3019909e7fa7b3482cdf9594556c2")
(setf (get state_estimator_msgs::getEstimatorDescriptionRequest :datatype-) "state_estimator_msgs/getEstimatorDescriptionRequest")
(setf (get state_estimator_msgs::getEstimatorDescriptionRequest :definition-)
      "string name
---
bool success
string description

")

(setf (get state_estimator_msgs::getEstimatorDescriptionResponse :md5sum-) "12c3019909e7fa7b3482cdf9594556c2")
(setf (get state_estimator_msgs::getEstimatorDescriptionResponse :datatype-) "state_estimator_msgs/getEstimatorDescriptionResponse")
(setf (get state_estimator_msgs::getEstimatorDescriptionResponse :definition-)
      "string name
---
bool success
string description

")



(provide :state_estimator_msgs/getEstimatorDescription "12c3019909e7fa7b3482cdf9594556c2")


