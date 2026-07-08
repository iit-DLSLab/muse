;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::listAllEstimators)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'listAllEstimators (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORS")
  (make-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORS"))
(unless (find-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORSREQUEST")
  (make-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORSREQUEST"))
(unless (find-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORSRESPONSE")
  (make-package "STATE_ESTIMATOR_MSGS::LISTALLESTIMATORSRESPONSE"))

(in-package "ROS")





(defclass state_estimator_msgs::listAllEstimatorsRequest
  :super ros::object
  :slots ())

(defmethod state_estimator_msgs::listAllEstimatorsRequest
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

(defclass state_estimator_msgs::listAllEstimatorsResponse
  :super ros::object
  :slots (_names ))

(defmethod state_estimator_msgs::listAllEstimatorsResponse
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

(defclass state_estimator_msgs::listAllEstimators
  :super ros::object
  :slots ())

(setf (get state_estimator_msgs::listAllEstimators :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::listAllEstimators :datatype-) "state_estimator_msgs/listAllEstimators")
(setf (get state_estimator_msgs::listAllEstimators :request) state_estimator_msgs::listAllEstimatorsRequest)
(setf (get state_estimator_msgs::listAllEstimators :response) state_estimator_msgs::listAllEstimatorsResponse)

(defmethod state_estimator_msgs::listAllEstimatorsRequest
  (:response () (instance state_estimator_msgs::listAllEstimatorsResponse :init)))

(setf (get state_estimator_msgs::listAllEstimatorsRequest :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::listAllEstimatorsRequest :datatype-) "state_estimator_msgs/listAllEstimatorsRequest")
(setf (get state_estimator_msgs::listAllEstimatorsRequest :definition-)
      "---
string[] names

")

(setf (get state_estimator_msgs::listAllEstimatorsResponse :md5sum-) "dc7ae3609524b18034e49294a4ce670e")
(setf (get state_estimator_msgs::listAllEstimatorsResponse :datatype-) "state_estimator_msgs/listAllEstimatorsResponse")
(setf (get state_estimator_msgs::listAllEstimatorsResponse :definition-)
      "---
string[] names

")



(provide :state_estimator_msgs/listAllEstimators "dc7ae3609524b18034e49294a4ce670e")


