; Auto-generated. Do not edit!


(cl:in-package anymal_msgs-msg)


;//! \htmlinclude AnymalState.msg.html

(cl:defclass <AnymalState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:TwistStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TwistStamped))
   (joints
    :reader joints
    :initarg :joints
    :type anymal_msgs-msg:ExtendedJointState
    :initform (cl:make-instance 'anymal_msgs-msg:ExtendedJointState))
   (contacts
    :reader contacts
    :initarg :contacts
    :type (cl:vector anymal_msgs-msg:Contact)
   :initform (cl:make-array 0 :element-type 'anymal_msgs-msg:Contact :initial-element (cl:make-instance 'anymal_msgs-msg:Contact)))
   (frame_transforms
    :reader frame_transforms
    :initarg :frame_transforms
    :type (cl:vector geometry_msgs-msg:TransformStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:TransformStamped :initial-element (cl:make-instance 'geometry_msgs-msg:TransformStamped))))
)

(cl:defclass AnymalState (<AnymalState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnymalState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnymalState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name anymal_msgs-msg:<AnymalState> is deprecated: use anymal_msgs-msg:AnymalState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:header-val is deprecated.  Use anymal_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:state-val is deprecated.  Use anymal_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:pose-val is deprecated.  Use anymal_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:twist-val is deprecated.  Use anymal_msgs-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:joints-val is deprecated.  Use anymal_msgs-msg:joints instead.")
  (joints m))

(cl:ensure-generic-function 'contacts-val :lambda-list '(m))
(cl:defmethod contacts-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:contacts-val is deprecated.  Use anymal_msgs-msg:contacts instead.")
  (contacts m))

(cl:ensure-generic-function 'frame_transforms-val :lambda-list '(m))
(cl:defmethod frame_transforms-val ((m <AnymalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:frame_transforms-val is deprecated.  Use anymal_msgs-msg:frame_transforms instead.")
  (frame_transforms m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AnymalState>)))
    "Constants for message type '<AnymalState>"
  '((:STATE_ERROR_SENSOR . -3)
    (:STATE_ERROR_ESTIMATOR . -2)
    (:STATE_ERROR_UNKNOWN . -1)
    (:STATE_OK . 0)
    (:STATE_UNINITIALIZED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AnymalState)))
    "Constants for message type 'AnymalState"
  '((:STATE_ERROR_SENSOR . -3)
    (:STATE_ERROR_ESTIMATOR . -2)
    (:STATE_ERROR_UNKNOWN . -1)
    (:STATE_OK . 0)
    (:STATE_UNINITIALIZED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnymalState>) ostream)
  "Serializes a message object of type '<AnymalState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joints) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'contacts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'contacts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frame_transforms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'frame_transforms))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnymalState>) istream)
  "Deserializes a message object of type '<AnymalState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joints) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'contacts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'contacts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'anymal_msgs-msg:Contact))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frame_transforms) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frame_transforms)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:TransformStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnymalState>)))
  "Returns string type for a message object of type '<AnymalState>"
  "anymal_msgs/AnymalState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnymalState)))
  "Returns string type for a message object of type 'AnymalState"
  "anymal_msgs/AnymalState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnymalState>)))
  "Returns md5sum for a message object of type '<AnymalState>"
  "1bee66925f9d04663ad1cd7f4a496f93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnymalState)))
  "Returns md5sum for a message object of type 'AnymalState"
  "1bee66925f9d04663ad1cd7f4a496f93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnymalState>)))
  "Returns full string definition for message of type '<AnymalState>"
  (cl:format cl:nil "# Anymal state including generalized coordinates, velocities, and contacts.~%std_msgs/Header header~%~%int8 STATE_ERROR_SENSOR=-3~%int8 STATE_ERROR_ESTIMATOR=-2~%int8 STATE_ERROR_UNKNOWN=-1~%int8 STATE_OK=0~%int8 STATE_UNINITIALIZED=1~%int8 state~%~%# Position of the base with respect to the odom frame, and orientation from base to odom.~%geometry_msgs/PoseStamped pose~%~%# Linear velocity and local angular velocity expressed in base frame.~%geometry_msgs/TwistStamped twist~%ExtendedJointState joints~%Contact[] contacts~%geometry_msgs/TransformStamped[] frame_transforms~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: anymal_msgs/ExtendedJointState~%# sensor_msgs/JointState with an additional field for acceleration.~%std_msgs/Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] acceleration~%float64[] effort~%~%================================================================================~%MSG: anymal_msgs/Contact~%# Contact definition.~%uint8 STATE_OPEN=0~%uint8 STATE_CLOSED=1~%uint8 STATE_SLIPPING=2~%~%std_msgs/Header header~%string name~%uint8 state~%geometry_msgs/Wrench wrench~%geometry_msgs/Point position~%geometry_msgs/Vector3 normal~%float64 frictionCoefficient~%float64 restitutionCoefficient~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnymalState)))
  "Returns full string definition for message of type 'AnymalState"
  (cl:format cl:nil "# Anymal state including generalized coordinates, velocities, and contacts.~%std_msgs/Header header~%~%int8 STATE_ERROR_SENSOR=-3~%int8 STATE_ERROR_ESTIMATOR=-2~%int8 STATE_ERROR_UNKNOWN=-1~%int8 STATE_OK=0~%int8 STATE_UNINITIALIZED=1~%int8 state~%~%# Position of the base with respect to the odom frame, and orientation from base to odom.~%geometry_msgs/PoseStamped pose~%~%# Linear velocity and local angular velocity expressed in base frame.~%geometry_msgs/TwistStamped twist~%ExtendedJointState joints~%Contact[] contacts~%geometry_msgs/TransformStamped[] frame_transforms~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: anymal_msgs/ExtendedJointState~%# sensor_msgs/JointState with an additional field for acceleration.~%std_msgs/Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] acceleration~%float64[] effort~%~%================================================================================~%MSG: anymal_msgs/Contact~%# Contact definition.~%uint8 STATE_OPEN=0~%uint8 STATE_CLOSED=1~%uint8 STATE_SLIPPING=2~%~%std_msgs/Header header~%string name~%uint8 state~%geometry_msgs/Wrench wrench~%geometry_msgs/Point position~%geometry_msgs/Vector3 normal~%float64 frictionCoefficient~%float64 restitutionCoefficient~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnymalState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joints))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'contacts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frame_transforms) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnymalState>))
  "Converts a ROS message object to a list"
  (cl:list 'AnymalState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':joints (joints msg))
    (cl:cons ':contacts (contacts msg))
    (cl:cons ':frame_transforms (frame_transforms msg))
))
