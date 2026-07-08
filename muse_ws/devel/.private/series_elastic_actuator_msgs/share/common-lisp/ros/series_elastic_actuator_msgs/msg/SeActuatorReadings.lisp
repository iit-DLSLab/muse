; Auto-generated. Do not edit!


(cl:in-package series_elastic_actuator_msgs-msg)


;//! \htmlinclude SeActuatorReadings.msg.html

(cl:defclass <SeActuatorReadings> (roslisp-msg-protocol:ros-message)
  ((readings
    :reader readings
    :initarg :readings
    :type (cl:vector series_elastic_actuator_msgs-msg:SeActuatorReading)
   :initform (cl:make-array 0 :element-type 'series_elastic_actuator_msgs-msg:SeActuatorReading :initial-element (cl:make-instance 'series_elastic_actuator_msgs-msg:SeActuatorReading))))
)

(cl:defclass SeActuatorReadings (<SeActuatorReadings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SeActuatorReadings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SeActuatorReadings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name series_elastic_actuator_msgs-msg:<SeActuatorReadings> is deprecated: use series_elastic_actuator_msgs-msg:SeActuatorReadings instead.")))

(cl:ensure-generic-function 'readings-val :lambda-list '(m))
(cl:defmethod readings-val ((m <SeActuatorReadings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:readings-val is deprecated.  Use series_elastic_actuator_msgs-msg:readings instead.")
  (readings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SeActuatorReadings>) ostream)
  "Serializes a message object of type '<SeActuatorReadings>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'readings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'readings))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SeActuatorReadings>) istream)
  "Deserializes a message object of type '<SeActuatorReadings>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'readings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'readings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'series_elastic_actuator_msgs-msg:SeActuatorReading))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SeActuatorReadings>)))
  "Returns string type for a message object of type '<SeActuatorReadings>"
  "series_elastic_actuator_msgs/SeActuatorReadings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SeActuatorReadings)))
  "Returns string type for a message object of type 'SeActuatorReadings"
  "series_elastic_actuator_msgs/SeActuatorReadings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SeActuatorReadings>)))
  "Returns md5sum for a message object of type '<SeActuatorReadings>"
  "ff6230194b1da4ad6213f9aae966106d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SeActuatorReadings)))
  "Returns md5sum for a message object of type 'SeActuatorReadings"
  "ff6230194b1da4ad6213f9aae966106d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SeActuatorReadings>)))
  "Returns full string definition for message of type '<SeActuatorReadings>"
  (cl:format cl:nil "# Series-elastic actuator readings~%~%SeActuatorReading[] readings~%~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorReading~%# Series-elastic actuator reading~%#~%Header header~%# States~%SeActuatorState state~%# Set-points~%SeActuatorCommand commanded~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorState~%# Series-elastic actuator state~%~%# Header of the message~%Header header~%# Name of the actuator~%string name~%# Statusword~%uint32 statusword~%# Motor current [A]~%float64 current~%# Position of the spring on the gear side [rad]~%float64 gear_position~%# Velocity of the spring on the gear side [rad/s]~%float64 gear_velocity~%# Position of the spring on the joint side [rad]~%float64 joint_position~%# Velocity of the spring on the joint side [rad/s]~%float64 joint_velocity~%# Acceleration of the spring on the joint side [rad/s^2]~%float64 joint_acceleration~%# Joint torque [Nm]~%float64 joint_torque~%# IMU measurement~%sensor_msgs/Imu imu~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorCommand~%# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SeActuatorReadings)))
  "Returns full string definition for message of type 'SeActuatorReadings"
  (cl:format cl:nil "# Series-elastic actuator readings~%~%SeActuatorReading[] readings~%~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorReading~%# Series-elastic actuator reading~%#~%Header header~%# States~%SeActuatorState state~%# Set-points~%SeActuatorCommand commanded~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorState~%# Series-elastic actuator state~%~%# Header of the message~%Header header~%# Name of the actuator~%string name~%# Statusword~%uint32 statusword~%# Motor current [A]~%float64 current~%# Position of the spring on the gear side [rad]~%float64 gear_position~%# Velocity of the spring on the gear side [rad/s]~%float64 gear_velocity~%# Position of the spring on the joint side [rad]~%float64 joint_position~%# Velocity of the spring on the joint side [rad/s]~%float64 joint_velocity~%# Acceleration of the spring on the joint side [rad/s^2]~%float64 joint_acceleration~%# Joint torque [Nm]~%float64 joint_torque~%# IMU measurement~%sensor_msgs/Imu imu~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: series_elastic_actuator_msgs/SeActuatorCommand~%# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SeActuatorReadings>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'readings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SeActuatorReadings>))
  "Converts a ROS message object to a list"
  (cl:list 'SeActuatorReadings
    (cl:cons ':readings (readings msg))
))
