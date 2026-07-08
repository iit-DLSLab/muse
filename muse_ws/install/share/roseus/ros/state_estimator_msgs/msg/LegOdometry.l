;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::LegOdometry)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'LegOdometry (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::LEGODOMETRY")
  (make-package "STATE_ESTIMATOR_MSGS::LEGODOMETRY"))

(in-package "ROS")
;;//! \htmlinclude LegOdometry.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass state_estimator_msgs::LegOdometry
  :super ros::object
  :slots (_header _lin_vel_lf _lin_vel_rf _lin_vel_lh _lin_vel_rh _base_velocity _stance_count _base_height_valid _base_height _stance _base_to_foot_position_world_z ))

(defmethod state_estimator_msgs::LegOdometry
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lin_vel_lf __lin_vel_lf) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_rf __lin_vel_rf) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_lh __lin_vel_lh) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:lin_vel_rh __lin_vel_rh) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:base_velocity __base_velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:stance_count __stance_count) 0)
    ((:base_height_valid __base_height_valid) nil)
    ((:base_height __base_height) 0.0)
    ((:stance __stance) (let (r) (dotimes (i 4) (push nil r)) r))
    ((:base_to_foot_position_world_z __base_to_foot_position_world_z) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lin_vel_lf __lin_vel_lf)
   (setq _lin_vel_rf __lin_vel_rf)
   (setq _lin_vel_lh __lin_vel_lh)
   (setq _lin_vel_rh __lin_vel_rh)
   (setq _base_velocity __base_velocity)
   (setq _stance_count (round __stance_count))
   (setq _base_height_valid __base_height_valid)
   (setq _base_height (float __base_height))
   (setq _stance __stance)
   (setq _base_to_foot_position_world_z __base_to_foot_position_world_z)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lin_vel_lf
   (&optional __lin_vel_lf)
   (if __lin_vel_lf (setq _lin_vel_lf __lin_vel_lf)) _lin_vel_lf)
  (:lin_vel_rf
   (&optional __lin_vel_rf)
   (if __lin_vel_rf (setq _lin_vel_rf __lin_vel_rf)) _lin_vel_rf)
  (:lin_vel_lh
   (&optional __lin_vel_lh)
   (if __lin_vel_lh (setq _lin_vel_lh __lin_vel_lh)) _lin_vel_lh)
  (:lin_vel_rh
   (&optional __lin_vel_rh)
   (if __lin_vel_rh (setq _lin_vel_rh __lin_vel_rh)) _lin_vel_rh)
  (:base_velocity
   (&optional __base_velocity)
   (if __base_velocity (setq _base_velocity __base_velocity)) _base_velocity)
  (:stance_count
   (&optional __stance_count)
   (if __stance_count (setq _stance_count __stance_count)) _stance_count)
  (:base_height_valid
   (&optional (__base_height_valid :null))
   (if (not (eq __base_height_valid :null)) (setq _base_height_valid __base_height_valid)) _base_height_valid)
  (:base_height
   (&optional __base_height)
   (if __base_height (setq _base_height __base_height)) _base_height)
  (:stance
   (&optional __stance)
   (if __stance (setq _stance __stance)) _stance)
  (:base_to_foot_position_world_z
   (&optional __base_to_foot_position_world_z)
   (if __base_to_foot_position_world_z (setq _base_to_foot_position_world_z __base_to_foot_position_world_z)) _base_to_foot_position_world_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _lin_vel_lf
    (* 8    3)
    ;; float64[3] _lin_vel_rf
    (* 8    3)
    ;; float64[3] _lin_vel_lh
    (* 8    3)
    ;; float64[3] _lin_vel_rh
    (* 8    3)
    ;; float64[3] _base_velocity
    (* 8    3)
    ;; uint8 _stance_count
    1
    ;; bool _base_height_valid
    1
    ;; float64 _base_height
    8
    ;; bool[4] _stance
    (* 1    4)
    ;; float64[4] _base_to_foot_position_world_z
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _lin_vel_lf
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_lf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_rf
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_rf i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_lh
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_lh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _lin_vel_rh
     (dotimes (i 3)
       (sys::poke (elt _lin_vel_rh i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _base_velocity
     (dotimes (i 3)
       (sys::poke (elt _base_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _stance_count
       (write-byte _stance_count s)
     ;; bool _base_height_valid
       (if _base_height_valid (write-byte -1 s) (write-byte 0 s))
     ;; float64 _base_height
       (sys::poke _base_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool[4] _stance
     (dotimes (i 4)
       (if (elt _stance i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; float64[4] _base_to_foot_position_world_z
     (dotimes (i 4)
       (sys::poke (elt _base_to_foot_position_world_z i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _lin_vel_lf
   (dotimes (i (length _lin_vel_lf))
     (setf (elt _lin_vel_lf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_rf
   (dotimes (i (length _lin_vel_rf))
     (setf (elt _lin_vel_rf i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_lh
   (dotimes (i (length _lin_vel_lh))
     (setf (elt _lin_vel_lh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _lin_vel_rh
   (dotimes (i (length _lin_vel_rh))
     (setf (elt _lin_vel_rh i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _base_velocity
   (dotimes (i (length _base_velocity))
     (setf (elt _base_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint8 _stance_count
     (setq _stance_count (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _base_height_valid
     (setq _base_height_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _base_height
     (setq _base_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool[4] _stance
   (dotimes (i (length _stance))
     (setf (elt _stance i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; float64[4] _base_to_foot_position_world_z
   (dotimes (i (length _base_to_foot_position_world_z))
     (setf (elt _base_to_foot_position_world_z i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get state_estimator_msgs::LegOdometry :md5sum-) "569661be22a02d9165c8c394eeb247a7")
(setf (get state_estimator_msgs::LegOdometry :datatype-) "state_estimator_msgs/LegOdometry")
(setf (get state_estimator_msgs::LegOdometry :definition-)
      "std_msgs/Header header

float64[3] lin_vel_lf
float64[3] lin_vel_rf
float64[3] lin_vel_lh
float64[3] lin_vel_rh

float64[3] base_velocity

uint8 stance_count
bool base_height_valid
float64 base_height
bool[4] stance
float64[4] base_to_foot_position_world_z

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



(provide :state_estimator_msgs/LegOdometry "569661be22a02d9165c8c394eeb247a7")


