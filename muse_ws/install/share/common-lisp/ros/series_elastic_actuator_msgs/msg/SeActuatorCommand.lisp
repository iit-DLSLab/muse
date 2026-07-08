; Auto-generated. Do not edit!


(cl:in-package series_elastic_actuator_msgs-msg)


;//! \htmlinclude SeActuatorCommand.msg.html

(cl:defclass <SeActuatorCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (joint_torque
    :reader joint_torque
    :initarg :joint_torque
    :type cl:float
    :initform 0.0)
   (pid_gains_p
    :reader pid_gains_p
    :initarg :pid_gains_p
    :type cl:float
    :initform 0.0)
   (pid_gains_i
    :reader pid_gains_i
    :initarg :pid_gains_i
    :type cl:float
    :initform 0.0)
   (pid_gains_d
    :reader pid_gains_d
    :initarg :pid_gains_d
    :type cl:float
    :initform 0.0))
)

(cl:defclass SeActuatorCommand (<SeActuatorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SeActuatorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SeActuatorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name series_elastic_actuator_msgs-msg:<SeActuatorCommand> is deprecated: use series_elastic_actuator_msgs-msg:SeActuatorCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:header-val is deprecated.  Use series_elastic_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:name-val is deprecated.  Use series_elastic_actuator_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:mode-val is deprecated.  Use series_elastic_actuator_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:current-val is deprecated.  Use series_elastic_actuator_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:position-val is deprecated.  Use series_elastic_actuator_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:velocity-val is deprecated.  Use series_elastic_actuator_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'joint_torque-val :lambda-list '(m))
(cl:defmethod joint_torque-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:joint_torque-val is deprecated.  Use series_elastic_actuator_msgs-msg:joint_torque instead.")
  (joint_torque m))

(cl:ensure-generic-function 'pid_gains_p-val :lambda-list '(m))
(cl:defmethod pid_gains_p-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:pid_gains_p-val is deprecated.  Use series_elastic_actuator_msgs-msg:pid_gains_p instead.")
  (pid_gains_p m))

(cl:ensure-generic-function 'pid_gains_i-val :lambda-list '(m))
(cl:defmethod pid_gains_i-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:pid_gains_i-val is deprecated.  Use series_elastic_actuator_msgs-msg:pid_gains_i instead.")
  (pid_gains_i m))

(cl:ensure-generic-function 'pid_gains_d-val :lambda-list '(m))
(cl:defmethod pid_gains_d-val ((m <SeActuatorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader series_elastic_actuator_msgs-msg:pid_gains_d-val is deprecated.  Use series_elastic_actuator_msgs-msg:pid_gains_d instead.")
  (pid_gains_d m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SeActuatorCommand>)))
    "Constants for message type '<SeActuatorCommand>"
  '((:MODE_NA . 0)
    (:MODE_FREEZE . 1)
    (:MODE_DISABLE . 2)
    (:MODE_CURRENT . 3)
    (:MODE_MOTOR_POSITION . 4)
    (:MODE_MOTOR_VELOCITY . 5)
    (:MODE_GEAR_POSITION . 6)
    (:MODE_GEAR_VELOCITY . 7)
    (:MODE_JOINT_POSITION . 8)
    (:MODE_JOINT_VELOCITY . 9)
    (:MODE_JOINT_TORQUE . 10)
    (:MODE_JOINT_POSITION_VELOCITY . 11)
    (:MODE_JOINT_POSITION_VELOCITY_TORQUE . 12)
    (:MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SeActuatorCommand)))
    "Constants for message type 'SeActuatorCommand"
  '((:MODE_NA . 0)
    (:MODE_FREEZE . 1)
    (:MODE_DISABLE . 2)
    (:MODE_CURRENT . 3)
    (:MODE_MOTOR_POSITION . 4)
    (:MODE_MOTOR_VELOCITY . 5)
    (:MODE_GEAR_POSITION . 6)
    (:MODE_GEAR_VELOCITY . 7)
    (:MODE_JOINT_POSITION . 8)
    (:MODE_JOINT_VELOCITY . 9)
    (:MODE_JOINT_TORQUE . 10)
    (:MODE_JOINT_POSITION_VELOCITY . 11)
    (:MODE_JOINT_POSITION_VELOCITY_TORQUE . 12)
    (:MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SeActuatorCommand>) ostream)
  "Serializes a message object of type '<SeActuatorCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint_torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_gains_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_gains_i))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pid_gains_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SeActuatorCommand>) istream)
  "Deserializes a message object of type '<SeActuatorCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint_torque) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_gains_p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_gains_i) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pid_gains_d) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SeActuatorCommand>)))
  "Returns string type for a message object of type '<SeActuatorCommand>"
  "series_elastic_actuator_msgs/SeActuatorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SeActuatorCommand)))
  "Returns string type for a message object of type 'SeActuatorCommand"
  "series_elastic_actuator_msgs/SeActuatorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SeActuatorCommand>)))
  "Returns md5sum for a message object of type '<SeActuatorCommand>"
  "99caa130fb942b20d4b16f18464e90d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SeActuatorCommand)))
  "Returns md5sum for a message object of type 'SeActuatorCommand"
  "99caa130fb942b20d4b16f18464e90d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SeActuatorCommand>)))
  "Returns full string definition for message of type '<SeActuatorCommand>"
  (cl:format cl:nil "# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SeActuatorCommand)))
  "Returns full string definition for message of type 'SeActuatorCommand"
  (cl:format cl:nil "# Series-elastic actuator command~%~%Header header~%# Name of the actuator~%string name~%# Control mode~%int16 MODE_NA                                       = 0  # Not available~%int16 MODE_FREEZE                                   = 1  # Freeze motor~%int16 MODE_DISABLE                                  = 2  # Disable motor~%int16 MODE_CURRENT                                  = 3  # Track current~%int16 MODE_MOTOR_POSITION                           = 4  # Track motor position~%int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity~%int16 MODE_GEAR_POSITION                            = 6  # Track gear position~%int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity~%int16 MODE_JOINT_POSITION                           = 8  # Track joint position~%int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity~%int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque~%int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque~%int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains~%int16 mode~%# Motor current [A]~%float64 current~%# Position (motor/gear/joint) [rad]~%float64 position~%# Velocity (motor/gear/joint) [rad/s]~%float64 velocity~%# Torque (joint) [Nm]~%float64 joint_torque~%# Custom PID gains [-]~%float32 pid_gains_p~%float32 pid_gains_i~%float32 pid_gains_d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SeActuatorCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     2
     8
     8
     8
     8
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SeActuatorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'SeActuatorCommand
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':current (current msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':joint_torque (joint_torque msg))
    (cl:cons ':pid_gains_p (pid_gains_p msg))
    (cl:cons ':pid_gains_i (pid_gains_i msg))
    (cl:cons ':pid_gains_d (pid_gains_d msg))
))
