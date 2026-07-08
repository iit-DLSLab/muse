;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::getActiveEstimators)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'getActiveEstimators (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORS")
  (make-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORS"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORSREQUEST")
  (make-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORSREQUEST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORSRESPONSE")
  (make-package "STATE_ESTIMATOR_MSGS::GETACTIVEESTIMATORSRESPONSE"))

(in-package "ROS")





(defclass state_estimator_msgs::getActiveEstimatorsRequest
  :super ros::object
  :slots ())

(defmethod state_estimator_msgs::getActiveEstimatorsRequest
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

(defclass state_estimator_msgs::getActiveEstimatorsResponse
  :super ros::object
  :slots (_names ))

(defmethod state_estimator_msgs::getActiveEstimatorsResponse
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

(defclass state_estimator_msgs::getActiveEstimators
  :super ros::object
  :slots ())

(setf (get state_estimator_msgs::getActiveEstimators :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getActiveEstimators :datatype-) "state_estimator_msgs/getActiveEstimators")
(setf (get state_estimator_msgs::getActiveEstimators :request) state_estimator_msgs::getActiveEstimatorsRequest)
(setf (get state_estimator_msgs::getActiveEstimators :response) state_estimator_msgs::getActiveEstimatorsResponse)

(defmethod state_estimator_msgs::getActiveEstimatorsRequest
  (:response () (instance state_estimator_msgs::getActiveEstimatorsResponse :init)))

(setf (get state_estimator_msgs::getActiveEstimatorsRequest :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getActiveEstimatorsRequest :datatype-) "state_estimator_msgs/getActiveEstimatorsRequest")
(setf (get state_estimator_msgs::getActiveEstimatorsRequest :definition-)
      "---
string[] names

")

(setf (get state_estimator_msgs::getActiveEstimatorsResponse :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::getActiveEstimatorsResponse :datatype-) "state_estimator_msgs/getActiveEstimatorsResponse")
(setf (get state_estimator_msgs::getActiveEstimatorsResponse :definition-)
      "---
string[] names

")



(provide :state_estimator_msgs/getActiveEstimators "dc7ae3609524b18034e49294a4ce670e")


