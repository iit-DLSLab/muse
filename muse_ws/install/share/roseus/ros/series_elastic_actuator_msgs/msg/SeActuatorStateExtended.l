;; Auto-generated. Do not edit!


(when (boundp 'series_elastic_actuator_msgs::SeActuatorStateExtended)
  (if (not (find-package "SERIES_ELASTIC_ACTUATOR_MSGS"))
    (make-package "SERIES_ELASTIC_ACTUATOR_MSGS"))
  (shadow 'SeActuatorStateExtended (find-package "SERIES_ELASTIC_ACTUATOR_MSGS")))
(unless (find-package "SERIES_ELASTIC_ACTUATOR_MSGS::SEACTUATORSTATEEXTENDED")
  (make-package "SERIES_ELASTIC_ACTUATOR_MSGS::SEACTUATORSTATEEXTENDED"))

(in-package "ROS")
;;//! \htmlinclude SeActuatorStateExtended.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass series_elastic_actuator_msgs::SeActuatorStateExtended
  :super ros::object
  :slots (_header _name _statusword _current _gear_position _gear_velocity _joint_position _joint_velocity _joint_acceleration _joint_torque _imu _motor_position _motor_velocity _gear_position_ticks _joint_position_ticks _temperature _voltage _timestamp _desired_current_d _measured_current_d _desired_current_q _measured_current_q _measured_current_phase_u _measured_voltage_phase_u _measured_current_phase_v _measured_voltage_phase_v _measured_current_phase_w _measured_voltage_phase_w ))

(defmethod series_elastic_actuator_msgs::SeActuatorStateExtended
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
    ((:motor_position __motor_position) 0.0)
    ((:motor_velocity __motor_velocity) 0.0)
    ((:gear_position_ticks __gear_position_ticks) 0)
    ((:joint_position_ticks __joint_position_ticks) 0)
    ((:temperature __temperature) 0.0)
    ((:voltage __voltage) 0.0)
    ((:timestamp __timestamp) 0)
    ((:desired_current_d __desired_current_d) 0.0)
    ((:measured_current_d __measured_current_d) 0.0)
    ((:desired_current_q __desired_current_q) 0.0)
    ((:measured_current_q __measured_current_q) 0.0)
    ((:measured_current_phase_u __measured_current_phase_u) 0.0)
    ((:measured_voltage_phase_u __measured_voltage_phase_u) 0.0)
    ((:measured_current_phase_v __measured_current_phase_v) 0.0)
    ((:measured_voltage_phase_v __measured_voltage_phase_v) 0.0)
    ((:measured_current_phase_w __measured_current_phase_w) 0.0)
    ((:measured_voltage_phase_w __measured_voltage_phase_w) 0.0)
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
   (setq _motor_position (float __motor_position))
   (setq _motor_velocity (float __motor_velocity))
   (setq _gear_position_ticks (round __gear_position_ticks))
   (setq _joint_position_ticks (round __joint_position_ticks))
   (setq _temperature (float __temperature))
   (setq _voltage (float __voltage))
   (setq _timestamp (round __timestamp))
   (setq _desired_current_d (float __desired_current_d))
   (setq _measured_current_d (float __measured_current_d))
   (setq _desired_current_q (float __desired_current_q))
   (setq _measured_current_q (float __measured_current_q))
   (setq _measured_current_phase_u (float __measured_current_phase_u))
   (setq _measured_voltage_phase_u (float __measured_voltage_phase_u))
   (setq _measured_current_phase_v (float __measured_current_phase_v))
   (setq _measured_voltage_phase_v (float __measured_voltage_phase_v))
   (setq _measured_current_phase_w (float __measured_current_phase_w))
   (setq _measured_voltage_phase_w (float __measured_voltage_phase_w))
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
  (:motor_position
   (&optional __motor_position)
   (if __motor_position (setq _motor_position __motor_position)) _motor_position)
  (:motor_velocity
   (&optional __motor_velocity)
   (if __motor_velocity (setq _motor_velocity __motor_velocity)) _motor_velocity)
  (:gear_position_ticks
   (&optional __gear_position_ticks)
   (if __gear_position_ticks (setq _gear_position_ticks __gear_position_ticks)) _gear_position_ticks)
  (:joint_position_ticks
   (&optional __joint_position_ticks)
   (if __joint_position_ticks (setq _joint_position_ticks __joint_position_ticks)) _joint_position_ticks)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:desired_current_d
   (&optional __desired_current_d)
   (if __desired_current_d (setq _desired_current_d __desired_current_d)) _desired_current_d)
  (:measured_current_d
   (&optional __measured_current_d)
   (if __measured_current_d (setq _measured_current_d __measured_current_d)) _measured_current_d)
  (:desired_current_q
   (&optional __desired_current_q)
   (if __desired_current_q (setq _desired_current_q __desired_current_q)) _desired_current_q)
  (:measured_current_q
   (&optional __measured_current_q)
   (if __measured_current_q (setq _measured_current_q __measured_current_q)) _measured_current_q)
  (:measured_current_phase_u
   (&optional __measured_current_phase_u)
   (if __measured_current_phase_u (setq _measured_current_phase_u __measured_current_phase_u)) _measured_current_phase_u)
  (:measured_voltage_phase_u
   (&optional __measured_voltage_phase_u)
   (if __measured_voltage_phase_u (setq _measured_voltage_phase_u __measured_voltage_phase_u)) _measured_voltage_phase_u)
  (:measured_current_phase_v
   (&optional __measured_current_phase_v)
   (if __measured_current_phase_v (setq _measured_current_phase_v __measured_current_phase_v)) _measured_current_phase_v)
  (:measured_voltage_phase_v
   (&optional __measured_voltage_phase_v)
   (if __measured_voltage_phase_v (setq _measured_voltage_phase_v __measured_voltage_phase_v)) _measured_voltage_phase_v)
  (:measured_current_phase_w
   (&optional __measured_current_phase_w)
   (if __measured_current_phase_w (setq _measured_current_phase_w __measured_current_phase_w)) _measured_current_phase_w)
  (:measured_voltage_phase_w
   (&optional __measured_voltage_phase_w)
   (if __measured_voltage_phase_w (setq _measured_voltage_phase_w __measured_voltage_phase_w)) _measured_voltage_phase_w)
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
    ;; float64 _motor_position
    8
    ;; float64 _motor_velocity
    8
    ;; int32 _gear_position_ticks
    4
    ;; int32 _joint_position_ticks
    4
    ;; float64 _temperature
    8
    ;; float64 _voltage
    8
    ;; uint64 _timestamp
    8
    ;; float64 _desired_current_d
    8
    ;; float64 _measured_current_d
    8
    ;; float64 _desired_current_q
    8
    ;; float64 _measured_current_q
    8
    ;; float64 _measured_current_phase_u
    8
    ;; float64 _measured_voltage_phase_u
    8
    ;; float64 _measured_current_phase_v
    8
    ;; float64 _measured_voltage_phase_v
    8
    ;; float64 _measured_current_phase_w
    8
    ;; float64 _measured_voltage_phase_w
    8
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
     ;; float64 _motor_position
       (sys::poke _motor_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _motor_velocity
       (sys::poke _motor_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _gear_position_ticks
       (write-long _gear_position_ticks s)
     ;; int32 _joint_position_ticks
       (write-long _joint_position_ticks s)
     ;; float64 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint64 _timestamp
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp) (= (length (_timestamp . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp . bv) 1) -1) s))
             ((and (class _timestamp) (= (length (_timestamp . bv)) 1)) ;; big1
              (write-long (elt (_timestamp . bv) 0) s)
              (write-long (if (>= _timestamp 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp s)(write-long (if (>= _timestamp 0) 0 #xffffffff) s)))
     ;; float64 _desired_current_d
       (sys::poke _desired_current_d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_current_d
       (sys::poke _measured_current_d (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _desired_current_q
       (sys::poke _desired_current_q (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_current_q
       (sys::poke _measured_current_q (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_current_phase_u
       (sys::poke _measured_current_phase_u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_voltage_phase_u
       (sys::poke _measured_voltage_phase_u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_current_phase_v
       (sys::poke _measured_current_phase_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_voltage_phase_v
       (sys::poke _measured_voltage_phase_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_current_phase_w
       (sys::poke _measured_current_phase_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _measured_voltage_phase_w
       (sys::poke _measured_voltage_phase_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64 _motor_position
     (setq _motor_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _motor_velocity
     (setq _motor_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _gear_position_ticks
     (setq _gear_position_ticks (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint_position_ticks
     (setq _joint_position_ticks (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _temperature
     (setq _temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint64 _timestamp
     
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _desired_current_d
     (setq _desired_current_d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_current_d
     (setq _measured_current_d (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _desired_current_q
     (setq _desired_current_q (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_current_q
     (setq _measured_current_q (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_current_phase_u
     (setq _measured_current_phase_u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_voltage_phase_u
     (setq _measured_voltage_phase_u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_current_phase_v
     (setq _measured_current_phase_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_voltage_phase_v
     (setq _measured_voltage_phase_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_current_phase_w
     (setq _measured_current_phase_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _measured_voltage_phase_w
     (setq _measured_voltage_phase_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get series_elastic_actuator_msgs::SeActuatorStateExtended :md5sum-) "363d2497c1625484d8f3fe9fe046c107")
(setf (get series_elastic_actuator_msgs::SeActuatorStateExtended :datatype-) "series_elastic_actuator_msgs/SeActuatorStateExtended")
(setf (get series_elastic_actuator_msgs::SeActuatorStateExtended :definition-)
      "# Series-elastic actuator extended state

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

# Extended state data:

# Position of the motor [rad]
float64 motor_position
# Velocity of the motor [rad/s]
float64 motor_velocity
# Position of the spring on the gear side [ticks]
int32 gear_position_ticks
# Position of the spring on the joint side [ticks]
int32 joint_position_ticks
# Temperature of the actuator [deg C]
float64 temperature
# Voltage [V]
float64 voltage
# Timestamp
uint64 timestamp
# Desired current d part
float64 desired_current_d
# Measured current d part
float64 measured_current_d
# Desired current q part
float64 desired_current_q
# Measured current q part
float64 measured_current_q
# Measured current phase u
float64 measured_current_phase_u
# Measured voltage phase u
float64 measured_voltage_phase_u
# Measured current phase v
float64 measured_current_phase_v
# Measured voltage phase v
float64 measured_voltage_phase_v
# Measured current phase w
float64 measured_current_phase_w
# Measured voltage phase w
float64 measured_voltage_phase_w


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



(provide :series_elastic_actuator_msgs/SeActuatorStateExtended "363d2497c1625484d8f3fe9fe046c107")


