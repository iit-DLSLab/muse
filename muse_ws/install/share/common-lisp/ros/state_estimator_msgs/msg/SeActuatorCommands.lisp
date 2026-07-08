; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude SeActuatorCommands.msg.html

(cl:defclass <SeActuatorCommands> (roslisp-msg-protocol:ros-message)
  ((commands
    :reader commands
    :initarg :commands
    :type (cl:vector state_estimator_msgs-msg:SeActuatorCommand)
   :initform (cl:make-array 0 :element-type 'state_estimator_msgs-msg:SeActuatorCommand :initial-element (cl:make-instance 'state_estimator_msgs-msg:SeActuatorCommand))))
)

(cl:defclass SeActuatorCommands (<SeActuatorCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SeActuatorCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SeActuatorCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<SeActuatorCommands> is deprecated: use state_estimator_msgs-msg:SeActuatorCommands instead.")))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <SeActuatorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:commands-val is deprecated.  Use state_estimator_msgs-msg:commands instead.")
  (commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SeActuatorCommands>) ostream)
  "Serializes a message object of type '<SeActuatorCommands>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'commands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'commands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SeActuatorCommands>) istream)
  "Deserializes a message object of type '<SeActuatorCommands>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'commands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'commands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'state_estimator_msgs-msg:SeActuatorCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SeActuatorCommands>)))
  "Returns string type for a message object of type '<SeActuatorCommands>"
  "state_estimator_msgs/SeActuatorCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SeActuatorCommands)))
  "Returns string type for a message object of type 'SeActuatorCommands"
  "state_estimator_msgs/SeActuatorCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SeActuatorCommands>)))
  "Returns md5sum for a message object of type '<SeActuatorCommands>"
  "8bac105afcdb07f69773e72515936600")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SeActuatorCommands)))
  "Returns md5sum for a message object of type 'SeActuatorCommands"
  "8bac105afcdb07f69773e72515936600")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SeActuatorCommands>)))
  "Returns full string definition for message of type '<SeActuatorCommands>"
  (cl:format cl:nil "# Series-elastic actuator commands~%~%SeActuatorCommand[] commands~%~%================================================================================~%MSG: state_estimator_msgs/SeActuatorCommand~%# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SeActuatorCommands)))
  "Returns full string definition for message of type 'SeActuatorCommands"
  (cl:format cl:nil "# Series-elastic actuator commands~%~%SeActuatorCommand[] commands~%~%================================================================================~%MSG: state_estimator_msgs/SeActuatorCommand~%# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SeActuatorCommands>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SeActuatorCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'SeActuatorCommands
    (cl:cons ':commands (commands msg))
))
