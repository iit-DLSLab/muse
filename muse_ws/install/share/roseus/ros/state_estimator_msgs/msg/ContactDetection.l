;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::ContactDetection)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'ContactDetection (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::CONTACTDETECTION")
  (make-package "STATE_ESTIMATOR_MSGS::CONTACTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ContactDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::ContactDetection
  :super ros::object
  :slots (_header _stance_lf _stance_rf _stance_lh _stance_rh ))

(defmethod state_estimator_msgs::ContactDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:stance_lf __stance_lf) nil)
    ((:stance_rf __stance_rf) nil)
    ((:stance_lh __stance_lh) nil)
    ((:stance_rh __stance_rh) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _stance_lf __stance_lf)
   (setq _stance_rf __stance_rf)
   (setq _stance_lh __stance_lh)
   (setq _stance_rh __stance_rh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:stance_lf
   (&optional (__stance_lf :null))
   (if (not (eq __stance_lf :null)) (setq _stance_lf __stance_lf)) _stance_lf)
  (:stance_rf
   (&optional (__stance_rf :null))
   (if (not (eq __stance_rf :null)) (setq _stance_rf __stance_rf)) _stance_rf)
  (:stance_lh
   (&optional (__stance_lh :null))
   (if (not (eq __stance_lh :null)) (setq _stance_lh __stance_lh)) _stance_lh)
  (:stance_rh
   (&optional (__stance_rh :null))
   (if (not (eq __stance_rh :null)) (setq _stance_rh __stance_rh)) _stance_rh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _stance_lf
    1
    ;; bool _stance_rf
    1
    ;; bool _stance_lh
    1
    ;; bool _stance_rh
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _stance_lf
       (if _stance_lf (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_rf
       (if _stance_rf (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_lh
       (if _stance_lh (write-byte -1 s) (write-byte 0 s))
     ;; bool _stance_rh
       (if _stance_rh (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _stance_lf
     (setq _stance_lf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_rf
     (setq _stance_rf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_lh
     (setq _stance_lh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _stance_rh
     (setq _stance_rh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get state_estimator_msgs::ContactDetection :md5sum-) "78b512adaaaa7714d41f42af63ebb35b")
(setf (get state_estimator_msgs::ContactDetection :datatype-) "state_estimator_msgs/ContactDetection")
(setf (get state_estimator_msgs::ContactDetection :definition-)
      "std_msgs/Header header

bool stance_lf
bool stance_rf
bool stance_lh
bool stance_rh
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :state_estimator_msgs/ContactDetection "78b512adaaaa7714d41f42af63ebb35b")


