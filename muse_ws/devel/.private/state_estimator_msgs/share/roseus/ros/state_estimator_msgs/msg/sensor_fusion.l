;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::sensor_fusion)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'sensor_fusion (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::SENSOR_FUSION")
  (make-package "STATE_ESTIMATOR_MSGS::SENSOR_FUSION"))

(in-package "ROS")
;;//! \htmlinclude sensor_fusion.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::sensor_fusion
  :super ros::object
  :slots (_header _position _linear_velocity ))

(defmethod state_estimator_msgs::sensor_fusion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:linear_velocity __linear_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _linear_velocity __linear_velocity)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _position
    (* 8    3)
    ;; float64[3] _linear_velocity
    (* 8    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _linear_velocity
     (dotimes (i 3)
       (sys::poke (elt _linear_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _linear_velocity
   (dotimes (i (length _linear_velocity))
     (setf (elt _linear_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::sensor_fusion :md5sum-) "971096c93fdd3d0854784d51bb392049")
(setf (get state_estimator_msgs::sensor_fusion :datatype-) "state_estimator_msgs/sensor_fusion")
(setf (get state_estimator_msgs::sensor_fusion :definition-)
      "std_msgs/Header header

float64[3] position
float64[3] linear_velocity

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



(provide :state_estimator_msgs/sensor_fusion "971096c93fdd3d0854784d51bb392049")


