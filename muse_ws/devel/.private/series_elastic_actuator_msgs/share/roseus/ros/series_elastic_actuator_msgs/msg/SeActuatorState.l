;; Auto-generated. Do not edit!


(when (boundp 'series_elastic_actuator_msgs::SeActuatorState)
  (if (not (find-package "SERIES_ELASTIC_ACTUATOR_MSGS"))
    (make-package "SERIES_ELASTIC_ACTUATOR_MSGS"))
  (shadow 'SeActuatorState (find-package "SERIES_ELASTIC_ACTUATOR_MSGS")))
(unless (find-package "SERIES_ELASTIC_ACTUATOR_MSGS::SEACTUATORSTATE")
  (make-package "SERIES_ELASTIC_ACTUATOR_MSGS::SEACTUATORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SeActuatorState.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass series_elastic_actuator_msgs::SeActuatorState
  :super ros::object
  :slots (_header _name _statusword _current _gear_position _gear_velocity _joint_position _joint_velocity _joint_acceleration _joint_torque _imu ))

(defmethod series_elastic_actuator_msgs::SeActuatorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:statusword __statusword) 0)
    ((:current __current) 0.0)
    ((:gear_position __gear_position) 0.0)
    ((:gear_velocity __gear_velocity) 0.0)
    ((:joint_position __joint_position) 0.0)
    ((:joint_velocity __joint_velocity) 0.0)
    ((:joint_acceleration __joint_acceleration) 0.0)
    ((:joint_torque __joint_torque) 0.0)
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _statusword (round __statusword))
   (setq _current (float __current))
   (setq _gear_position (float __gear_position))
   (setq _gear_velocity (float __gear_velocity))
   (setq _joint_position (float __joint_position))
   (setq _joint_velocity (float __joint_velocity))
   (setq _joint_acceleration (float __joint_acceleration))
   (setq _joint_torque (float __joint_torque))
   (setq _imu __imu)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:statusword
   (&optional __statusword)
   (if __statusword (setq _statusword __statusword)) _statusword)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:gear_position
   (&optional __gear_position)
   (if __gear_position (setq _gear_position __gear_position)) _gear_position)
  (:gear_velocity
   (&optional __gear_velocity)
   (if __gear_velocity (setq _gear_velocity __gear_velocity)) _gear_velocity)
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:joint_acceleration
   (&optional __joint_acceleration)
   (if __joint_acceleration (setq _joint_acceleration __joint_acceleration)) _joint_acceleration)
  (:joint_torque
   (&optional __joint_torque)
   (if __joint_torque (setq _joint_torque __joint_torque)) _joint_torque)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint32 _statusword
    4
    ;; float64 _current
    8
    ;; float64 _gear_position
    8
    ;; float64 _gear_velocity
    8
    ;; float64 _joint_position
    8
    ;; float64 _joint_velocity
    8
    ;; float64 _joint_acceleration
    8
    ;; float64 _joint_torque
    8
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _statusword
       (write-long _statusword s)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gear_position
       (sys::poke _gear_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gear_velocity
       (sys::poke _gear_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_position
       (sys::poke _joint_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_velocity
       (sys::poke _joint_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_acceleration
       (sys::poke _joint_acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_torque
       (sys::poke _joint_torque (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _statusword
     (setq _statusword (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gear_position
     (setq _gear_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gear_velocity
     (setq _gear_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_position
     (setq _joint_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_velocity
     (setq _joint_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_acceleration
     (setq _joint_acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_torque
     (setq _joint_torque (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;;
   self)
  )

(setf (get series_elastic_actuator_msgs::SeActuatorState :md5sum-) "5beb885e629c734ee244b26cb1ffcc17")
(setf (get series_elastic_actuator_msgs::SeActuatorState :datatype-) "series_elastic_actuator_msgs/SeActuatorState")
(setf (get series_elastic_actuator_msgs::SeActuatorState :definition-)
      "# Series-elastic actuator state

# Header of the message
Header header
# Name of the actuator
string name
# Statusword
uint32 statusword
# Motor current [A]
float64 current
# Position of the spring on the gear side [rad]
float64 gear_position
# Velocity of the spring on the gear side [rad/s]
float64 gear_velocity
# Position of the spring on the joint side [rad]
float64 joint_position
# Velocity of the spring on the joint side [rad/s]
float64 joint_velocity
# Acceleration of the spring on the joint side [rad/s^2]
float64 joint_acceleration
# Joint torque [Nm]
float64 joint_torque
# IMU measurement
sensor_msgs/Imu imu

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

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :series_elastic_actuator_msgs/SeActuatorState "5beb885e629c734ee244b26cb1ffcc17")


