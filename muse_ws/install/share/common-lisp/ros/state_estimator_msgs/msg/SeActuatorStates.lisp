; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude SeActuatorStates.msg.html

(cl:defclass <SeActuatorStates> (roslisp-msg-protocol:ros-message)
  ((states
    :reader states
    :initarg :states
    :type (cl:vector state_estimator_msgs-msg:SeActuatorState)
   :initform (cl:make-array 0 :element-type 'state_estimator_msgs-msg:SeActuatorState :initial-element (cl:make-instance 'state_estimator_msgs-msg:SeActuatorState))))
)

(cl:defclass SeActuatorStates (<SeActuatorStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SeActuatorStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SeActuatorStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<SeActuatorStates> is deprecated: use state_estimator_msgs-msg:SeActuatorStates instead.")))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <SeActuatorStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:states-val is deprecated.  Use state_estimator_msgs-msg:states instead.")
  (states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SeActuatorStates>) ostream)
  "Serializes a message object of type '<SeActuatorStates>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SeActuatorStates>) istream)
  "Deserializes a message object of type '<SeActuatorStates>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'state_estimator_msgs-msg:SeActuatorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SeActuatorStates>)))
  "Returns string type for a message object of type '<SeActuatorStates>"
  "state_estimator_msgs/SeActuatorStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SeActuatorStates)))
  "Returns string type for a message object of type 'SeActuatorStates"
  "state_estimator_msgs/SeActuatorStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SeActuatorStates>)))
  "Returns md5sum for a message object of type '<SeActuatorStates>"
  "5843775048a17145d12a3446d15a0bd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SeActuatorStates)))
  "Returns md5sum for a message object of type 'SeActuatorStates"
  "5843775048a17145d12a3446d15a0bd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SeActuatorStates>)))
  "Returns full string definition for message of type '<SeActuatorStates>"
  (cl:format cl:nil "# Series-elastic actuator states~%~%SeActuatorState[] states~%~%================================================================================~%MSG: state_estimator_msgs/SeActuatorState~%# Series-elastic actuator state~%~%# Header of the message~%Header header~%# Name of the actuator~%string name~%# Statusword~%uint32 statusword~%# Motor current [A]~%float64 current~%# Position of the spring on the gear side [rad]~%float64 gear_position~%# Velocity of the spring on the gear side [rad/s]~%float64 gear_velocity~%# Position of the spring on the joint side [rad]~%float64 joint_position~%# Velocity of the spring on the joint side [rad/s]~%float64 joint_velocity~%# Acceleration of the spring on the joint side [rad/s^2]~%float64 joint_acceleration~%# Joint torque [Nm]~%float64 joint_torque~%# IMU measurement~%sensor_msgs/Imu imu~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SeActuatorStates)))
  "Returns full string definition for message of type 'SeActuatorStates"
  (cl:format cl:nil "# Series-elastic actuator states~%~%SeActuatorState[] states~%~%================================================================================~%MSG: state_estimator_msgs/SeActuatorState~%# Series-elastic actuator state~%~%# Header of the message~%Header header~%# Name of the actuator~%string name~%# Statusword~%uint32 statusword~%# Motor current [A]~%float64 current~%# Position of the spring on the gear side [rad]~%float64 gear_position~%# Velocity of the spring on the gear side [rad/s]~%float64 gear_velocity~%# Position of the spring on the joint side [rad]~%float64 joint_position~%# Velocity of the spring on the joint side [rad/s]~%float64 joint_velocity~%# Acceleration of the spring on the joint side [rad/s^2]~%float64 joint_acceleration~%# Joint torque [Nm]~%float64 joint_torque~%# IMU measurement~%sensor_msgs/Imu imu~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SeActuatorStates>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SeActuatorStates>))
  "Converts a ROS message object to a list"
  (cl:list 'SeActuatorStates
    (cl:cons ':states (states msg))
))
