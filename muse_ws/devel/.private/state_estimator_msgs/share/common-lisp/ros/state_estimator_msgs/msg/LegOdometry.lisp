; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-msg)


;//! \htmlinclude LegOdometry.msg.html

(cl:defclass <LegOdometry> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lin_vel_lf
    :reader lin_vel_lf
    :initarg :lin_vel_lf
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_rf
    :reader lin_vel_rf
    :initarg :lin_vel_rf
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_lh
    :reader lin_vel_lh
    :initarg :lin_vel_lh
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (lin_vel_rh
    :reader lin_vel_rh
    :initarg :lin_vel_rh
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (base_velocity
    :reader base_velocity
    :initarg :base_velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (stance_count
    :reader stance_count
    :initarg :stance_count
    :type cl:fixnum
    :initform 0)
   (base_height_valid
    :reader base_height_valid
    :initarg :base_height_valid
    :type cl:boolean
    :initform cl:nil)
   (base_height
    :reader base_height
    :initarg :base_height
    :type cl:float
    :initform 0.0)
   (stance
    :reader stance
    :initarg :stance
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil))
   (base_to_foot_position_world_z
    :reader base_to_foot_position_world_z
    :initarg :base_to_foot_position_world_z
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass LegOdometry (<LegOdometry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegOdometry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegOdometry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-msg:<LegOdometry> is deprecated: use state_estimator_msgs-msg:LegOdometry instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:header-val is deprecated.  Use state_estimator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lin_vel_lf-val :lambda-list '(m))
(cl:defmethod lin_vel_lf-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_lf-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_lf instead.")
  (lin_vel_lf m))

(cl:ensure-generic-function 'lin_vel_rf-val :lambda-list '(m))
(cl:defmethod lin_vel_rf-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_rf-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_rf instead.")
  (lin_vel_rf m))

(cl:ensure-generic-function 'lin_vel_lh-val :lambda-list '(m))
(cl:defmethod lin_vel_lh-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_lh-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_lh instead.")
  (lin_vel_lh m))

(cl:ensure-generic-function 'lin_vel_rh-val :lambda-list '(m))
(cl:defmethod lin_vel_rh-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:lin_vel_rh-val is deprecated.  Use state_estimator_msgs-msg:lin_vel_rh instead.")
  (lin_vel_rh m))

(cl:ensure-generic-function 'base_velocity-val :lambda-list '(m))
(cl:defmethod base_velocity-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:base_velocity-val is deprecated.  Use state_estimator_msgs-msg:base_velocity instead.")
  (base_velocity m))

(cl:ensure-generic-function 'stance_count-val :lambda-list '(m))
(cl:defmethod stance_count-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance_count-val is deprecated.  Use state_estimator_msgs-msg:stance_count instead.")
  (stance_count m))

(cl:ensure-generic-function 'base_height_valid-val :lambda-list '(m))
(cl:defmethod base_height_valid-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:base_height_valid-val is deprecated.  Use state_estimator_msgs-msg:base_height_valid instead.")
  (base_height_valid m))

(cl:ensure-generic-function 'base_height-val :lambda-list '(m))
(cl:defmethod base_height-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:base_height-val is deprecated.  Use state_estimator_msgs-msg:base_height instead.")
  (base_height m))

(cl:ensure-generic-function 'stance-val :lambda-list '(m))
(cl:defmethod stance-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:stance-val is deprecated.  Use state_estimator_msgs-msg:stance instead.")
  (stance m))

(cl:ensure-generic-function 'base_to_foot_position_world_z-val :lambda-list '(m))
(cl:defmethod base_to_foot_position_world_z-val ((m <LegOdometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-msg:base_to_foot_position_world_z-val is deprecated.  Use state_estimator_msgs-msg:base_to_foot_position_world_z instead.")
  (base_to_foot_position_world_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegOdometry>) ostream)
  "Serializes a message object of type '<LegOdometry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_lf))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_rf))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_lh))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lin_vel_rh))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'base_velocity))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stance_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'base_height_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'base_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'stance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'base_to_foot_position_world_z))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegOdometry>) istream)
  "Deserializes a message object of type '<LegOdometry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'lin_vel_lf) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_lf)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lin_vel_rf) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_rf)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lin_vel_lh) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_lh)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lin_vel_rh) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'lin_vel_rh)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'base_velocity) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'base_velocity)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stance_count)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_height_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'base_height) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'stance) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'stance)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'base_to_foot_position_world_z) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'base_to_foot_position_world_z)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegOdometry>)))
  "Returns string type for a message object of type '<LegOdometry>"
  "state_estimator_msgs/LegOdometry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegOdometry)))
  "Returns string type for a message object of type 'LegOdometry"
  "state_estimator_msgs/LegOdometry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegOdometry>)))
  "Returns md5sum for a message object of type '<LegOdometry>"
  "569661be22a02d9165c8c394eeb247a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegOdometry)))
  "Returns md5sum for a message object of type 'LegOdometry"
  "569661be22a02d9165c8c394eeb247a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegOdometry>)))
  "Returns full string definition for message of type '<LegOdometry>"
  (cl:format cl:nil "std_msgs/Header header~%~%float64[3] lin_vel_lf~%float64[3] lin_vel_rf~%float64[3] lin_vel_lh~%float64[3] lin_vel_rh~%~%float64[3] base_velocity~%~%uint8 stance_count~%bool base_height_valid~%float64 base_height~%bool[4] stance~%float64[4] base_to_foot_position_world_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegOdometry)))
  "Returns full string definition for message of type 'LegOdometry"
  (cl:format cl:nil "std_msgs/Header header~%~%float64[3] lin_vel_lf~%float64[3] lin_vel_rf~%float64[3] lin_vel_lh~%float64[3] lin_vel_rh~%~%float64[3] base_velocity~%~%uint8 stance_count~%bool base_height_valid~%float64 base_height~%bool[4] stance~%float64[4] base_to_foot_position_world_z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegOdometry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_lf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_rf) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_lh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lin_vel_rh) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'base_velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'stance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'base_to_foot_position_world_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegOdometry>))
  "Converts a ROS message object to a list"
  (cl:list 'LegOdometry
    (cl:cons ':header (header msg))
    (cl:cons ':lin_vel_lf (lin_vel_lf msg))
    (cl:cons ':lin_vel_rf (lin_vel_rf msg))
    (cl:cons ':lin_vel_lh (lin_vel_lh msg))
    (cl:cons ':lin_vel_rh (lin_vel_rh msg))
    (cl:cons ':base_velocity (base_velocity msg))
    (cl:cons ':stance_count (stance_count msg))
    (cl:cons ':base_height_valid (base_height_valid msg))
    (cl:cons ':base_height (base_height msg))
    (cl:cons ':stance (stance msg))
    (cl:cons ':base_to_foot_position_world_z (base_to_foot_position_world_z msg))
))
