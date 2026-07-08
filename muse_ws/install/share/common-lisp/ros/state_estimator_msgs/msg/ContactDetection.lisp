; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude ContactDetection.msg.html

(cl:defclass <ContactDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (stance_lf
    :reader stance_lf
    :initarg :stance_lf
    :type cl:boolean
    :initform cl:nil)
   (stance_rf
    :reader stance_rf
    :initarg :stance_rf
    :type cl:boolean
    :initform cl:nil)
   (stance_lh
    :reader stance_lh
    :initarg :stance_lh
    :type cl:boolean
    :initform cl:nil)
   (stance_rh
    :reader stance_rh
    :initarg :stance_rh
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ContactDetection (<ContactDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ContactDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ContactDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<ContactDetection> is deprecated: use state_estimator_msgs-msg:ContactDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ContactDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:header-val is deprecated.  Use state_estimator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'stance_lf-val :lambda-list '(m))
(cl:defmethod stance_lf-val ((m <ContactDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_lf-val is deprecated.  Use state_estimator_msgs-msg:stance_lf instead.")
  (stance_lf m))

(cl:ensure-generic-function 'stance_rf-val :lambda-list '(m))
(cl:defmethod stance_rf-val ((m <ContactDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_rf-val is deprecated.  Use state_estimator_msgs-msg:stance_rf instead.")
  (stance_rf m))

(cl:ensure-generic-function 'stance_lh-val :lambda-list '(m))
(cl:defmethod stance_lh-val ((m <ContactDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_lh-val is deprecated.  Use state_estimator_msgs-msg:stance_lh instead.")
  (stance_lh m))

(cl:ensure-generic-function 'stance_rh-val :lambda-list '(m))
(cl:defmethod stance_rh-val ((m <ContactDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_rh-val is deprecated.  Use state_estimator_msgs-msg:stance_rh instead.")
  (stance_rh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ContactDetection>) ostream)
  "Serializes a message object of type '<ContactDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_lf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_rf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_lh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stance_rh) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ContactDetection>) istream)
  "Deserializes a message object of type '<ContactDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'stance_lf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_rf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_lh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stance_rh) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ContactDetection>)))
  "Returns string type for a message object of type '<ContactDetection>"
  "state_estimator_msgs/ContactDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ContactDetection)))
  "Returns string type for a message object of type 'ContactDetection"
  "state_estimator_msgs/ContactDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ContactDetection>)))
  "Returns md5sum for a message object of type '<ContactDetection>"
  "78b512adaaaa7714d41f42af63ebb35b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ContactDetection)))
  "Returns md5sum for a message object of type 'ContactDetection"
  "78b512adaaaa7714d41f42af63ebb35b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ContactDetection>)))
  "Returns full string definition for message of type '<ContactDetection>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool stance_lf~%bool stance_rf~%bool stance_lh~%bool stance_rh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ContactDetection)))
  "Returns full string definition for message of type 'ContactDetection"
  (cl:format cl:nil "std_msgs/Header header~%~%bool stance_lf~%bool stance_rf~%bool stance_lh~%bool stance_rh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ContactDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ContactDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'ContactDetection
    (cl:cons ':header (header msg))
    (cl:cons ':stance_lf (stance_lf msg))
    (cl:cons ':stance_rf (stance_rf msg))
    (cl:cons ':stance_lh (stance_lh msg))
    (cl:cons ':stance_rh (stance_rh msg))
))
