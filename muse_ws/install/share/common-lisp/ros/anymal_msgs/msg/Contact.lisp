; Auto-generated. Do not edit!


(cl:in-package anymal_msgs-msg)


;//! \htmlinclude Contact.msg.html

(cl:defclass <Contact> (roslisp-msg-protocol:ros-message)
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
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (normal
    :reader normal
    :initarg :normal
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (frictionCoefficient
    :reader frictionCoefficient
    :initarg :frictionCoefficient
    :type cl:float
    :initform 0.0)
   (restitutionCoefficient
    :reader restitutionCoefficient
    :initarg :restitutionCoefficient
    :type cl:float
    :initform 0.0))
)

(cl:defclass Contact (<Contact>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contact>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contact)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name anymal_msgs-msg:<Contact> is deprecated: use anymal_msgs-msg:Contact instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:header-val is deprecated.  Use anymal_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:name-val is deprecated.  Use anymal_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:state-val is deprecated.  Use anymal_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:wrench-val is deprecated.  Use anymal_msgs-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:position-val is deprecated.  Use anymal_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'normal-val :lambda-list '(m))
(cl:defmethod normal-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:normal-val is deprecated.  Use anymal_msgs-msg:normal instead.")
  (normal m))

(cl:ensure-generic-function 'frictionCoefficient-val :lambda-list '(m))
(cl:defmethod frictionCoefficient-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:frictionCoefficient-val is deprecated.  Use anymal_msgs-msg:frictionCoefficient instead.")
  (frictionCoefficient m))

(cl:ensure-generic-function 'restitutionCoefficient-val :lambda-list '(m))
(cl:defmethod restitutionCoefficient-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader anymal_msgs-msg:restitutionCoefficient-val is deprecated.  Use anymal_msgs-msg:restitutionCoefficient instead.")
  (restitutionCoefficient m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Contact>)))
    "Constants for message type '<Contact>"
  '((:STATE_OPEN . 0)
    (:STATE_CLOSED . 1)
    (:STATE_SLIPPING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Contact)))
    "Constants for message type 'Contact"
  '((:STATE_OPEN . 0)
    (:STATE_CLOSED . 1)
    (:STATE_SLIPPING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contact>) ostream)
  "Serializes a message object of type '<Contact>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'normal) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frictionCoefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'restitutionCoefficient))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contact>) istream)
  "Deserializes a message object of type '<Contact>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'normal) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frictionCoefficient) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'restitutionCoefficient) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contact>)))
  "Returns string type for a message object of type '<Contact>"
  "anymal_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contact)))
  "Returns string type for a message object of type 'Contact"
  "anymal_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contact>)))
  "Returns md5sum for a message object of type '<Contact>"
  "48c55012030000155d5ac40f7586c2e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contact)))
  "Returns md5sum for a message object of type 'Contact"
  "48c55012030000155d5ac40f7586c2e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contact>)))
  "Returns full string definition for message of type '<Contact>"
  (cl:format cl:nil "# Contact definition.~%uint8 STATE_OPEN=0~%uint8 STATE_CLOSED=1~%uint8 STATE_SLIPPING=2~%~%std_msgs/Header header~%string name~%uint8 state~%geometry_msgs/Wrench wrench~%geometry_msgs/Point position~%geometry_msgs/Vector3 normal~%float64 frictionCoefficient~%float64 restitutionCoefficient~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contact)))
  "Returns full string definition for message of type 'Contact"
  (cl:format cl:nil "# Contact definition.~%uint8 STATE_OPEN=0~%uint8 STATE_CLOSED=1~%uint8 STATE_SLIPPING=2~%~%std_msgs/Header header~%string name~%uint8 state~%geometry_msgs/Wrench wrench~%geometry_msgs/Point position~%geometry_msgs/Vector3 normal~%float64 frictionCoefficient~%float64 restitutionCoefficient~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contact>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'normal))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contact>))
  "Converts a ROS message object to a list"
  (cl:list 'Contact
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':state (state msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':position (position msg))
    (cl:cons ':normal (normal msg))
    (cl:cons ':frictionCoefficient (frictionCoefficient msg))
    (cl:cons ':restitutionCoefficient (restitutionCoefficient msg))
))
