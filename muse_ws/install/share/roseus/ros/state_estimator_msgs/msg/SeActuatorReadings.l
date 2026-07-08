;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::SeActuatorReadings)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'SeActuatorReadings (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORREADINGS")
  (make-package "STATE_ESTIMATOR_MSGS::SEACTUATORREADINGS"))

(in-package "ROS")
;;//! \htmlinclude SeActuatorReadings.msg.html


(defclass state_estimator_msgs::SeActuatorReadings
  :super ros::object
  :slots (_readings ))

(defmethod state_estimator_msgs::SeActuatorReadings
  (:init
   (&key
    ((:readings __readings) ())
    )
   (send-super :init)
   (setq _readings __readings)
   self)
  (:readings
   (&rest __readings)
   (if (keywordp (car __readings))
       (send* _readings __readings)
     (progn
       (if __readings (setq _readings (car __readings)))
       _readings)))
  (:serialization-length
   ()
   (+
    ;; state_estimator_msgs/SeActuatorReading[] _readings
    (apply #'+ (send-all _readings :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; state_estimator_msgs/SeActuatorReading[] _readings
     (write-long (length _readings) s)
     (dolist (elem _readings)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; state_estimator_msgs/SeActuatorReading[] _readings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _readings (let (r) (dotimes (i n) (push (instance state_estimator_msgs::SeActuatorReading :init) r)) r))
     (dolist (elem- _readings)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get state_estimator_msgs::SeActuatorReadings :md5sum-) "ff6230194b1da4ad6213f9aae966106d")
(setf (get state_estimator_msgs::SeActuatorReadings :datatype-) "state_estimator_msgs/SeActuatorReadings")
(setf (get state_estimator_msgs::SeActuatorReadings :definition-)
      "# Series-elastic actuator readings

SeActuatorReading[] readings

================================================================================
MSG: state_estimator_msgs/SeActuatorReading
# Series-elastic actuator reading
#
Header header
# States
SeActuatorState state
# Set-points
SeActuatorCommand commanded


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
MSG: state_estimator_msgs/SeActuatorState
# Series-elastic actuator state

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
================================================================================
MSG: state_estimator_msgs/SeActuatorCommand
# Series-elastic actuator command

Header header
# Name of the actuator
string name
# Control mode
int16 MODE_NA                                       = 0  # Not available
int16 MODE_FREEZE                                   = 1  # Freeze motor
int16 MODE_DISABLE                                  = 2  # Disable motor
int16 MODE_CURRENT                                  = 3  # Track current
int16 MODE_MOTOR_POSITION                           = 4  # Track motor position
int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity
int16 MODE_GEAR_POSITION                            = 6  # Track gear position
int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity
int16 MODE_JOINT_POSITION                           = 8  # Track joint position
int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity
int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque
int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity
int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque
int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains
int16 mode
# Motor current [A]
float64 current
# Position (motor/gear/joint) [rad]
float64 position
# Velocity (motor/gear/joint) [rad/s]
float64 velocity
# Torque (joint) [Nm]
float64 joint_torque
# Custom PID gains [-]
float32 pid_gains_p
float32 pid_gains_i
float32 pid_gains_d

")



(provide :state_estimator_msgs/SeActuatorReadings "ff6230194b1da4ad6213f9aae966106d")


