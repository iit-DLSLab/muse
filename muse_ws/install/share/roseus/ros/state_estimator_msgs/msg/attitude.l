;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::attitude)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'attitude (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::ATTITUDE")
  (make-package "STATE_ESTIMATOR_MSGS::ATTITUDE"))

(in-package "ROS")
;;//! \htmlinclude attitude.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::attitude
  :super ros::object
  :slots (_header _quaternion _roll_deg _pitch_deg _yaw_deg _angular_velocity ))

(defmethod state_estimator_msgs::attitude
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quaternion __quaternion) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:roll_deg __roll_deg) 0.0)
    ((:pitch_deg __pitch_deg) 0.0)
    ((:yaw_deg __yaw_deg) 0.0)
    ((:angular_velocity __angular_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _quaternion __quaternion)
   (setq _roll_deg (float __roll_deg))
   (setq _pitch_deg (float __pitch_deg))
   (setq _yaw_deg (float __yaw_deg))
   (setq _angular_velocity __angular_velocity)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quaternion
   (&optional __quaternion)
   (if __quaternion (setq _quaternion __quaternion)) _quaternion)
  (:roll_deg
   (&optional __roll_deg)
   (if __roll_deg (setq _roll_deg __roll_deg)) _roll_deg)
  (:pitch_deg
   (&optional __pitch_deg)
   (if __pitch_deg (setq _pitch_deg __pitch_deg)) _pitch_deg)
  (:yaw_deg
   (&optional __yaw_deg)
   (if __yaw_deg (setq _yaw_deg __yaw_deg)) _yaw_deg)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[4] _quaternion
    (* 8    4)
    ;; float64 _roll_deg
    8
    ;; float64 _pitch_deg
    8
    ;; float64 _yaw_deg
    8
    ;; float64[3] _angular_velocity
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[4] _quaternion
     (dotimes (i 4)
       (sys::poke (elt _quaternion i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _roll_deg
       (sys::poke _roll_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch_deg
       (sys::poke _pitch_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_deg
       (sys::poke _yaw_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _angular_velocity
     (dotimes (i 3)
       (sys::poke (elt _angular_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[4] _quaternion
   (dotimes (i (length _quaternion))
     (setf (elt _quaternion i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _roll_deg
     (setq _roll_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch_deg
     (setq _pitch_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_deg
     (setq _yaw_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _angular_velocity
   (dotimes (i (length _angular_velocity))
     (setf (elt _angular_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::attitude :md5sum-) "ecef9a2429ca3b3adda042fe513a909a")
(setf (get state_estimator_msgs::attitude :datatype-) "state_estimator_msgs/attitude")
(setf (get state_estimator_msgs::attitude :definition-)
      "std_msgs/Header header

float64[4] quaternion
float64 roll_deg
float64 pitch_deg
float64 yaw_deg 
float64[3] angular_velocity
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



(provide :state_estimator_msgs/attitude "ecef9a2429ca3b3adda042fe513a909a")


