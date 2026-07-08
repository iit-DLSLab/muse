; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude getBlacklist-request.msg.html

(cl:defclass <getBlacklist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getBlacklist-request (<getBlacklist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getBlacklist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getBlacklist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getBlacklist-request> is deprecated: use state_estimator_msgs-srv:getBlacklist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getBlacklist-request>) ostream)
  "Serializes a message object of type '<getBlacklist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getBlacklist-request>) istream)
  "Deserializes a message object of type '<getBlacklist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getBlacklist-request>)))
  "Returns string type for a service object of type '<getBlacklist-request>"
  "state_estimator_msgs/getBlacklistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getBlacklist-request)))
  "Returns string type for a service object of type 'getBlacklist-request"
  "state_estimator_msgs/getBlacklistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getBlacklist-request>)))
  "Returns md5sum for a message object of type '<getBlacklist-request>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getBlacklist-request)))
  "Returns md5sum for a message object of type 'getBlacklist-request"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getBlacklist-request>)))
  "Returns full string definition for message of type '<getBlacklist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getBlacklist-request)))
  "Returns full string definition for message of type 'getBlacklist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getBlacklist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getBlacklist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getBlacklist-request
))
;//! \htmlinclude getBlacklist-response.msg.html

(cl:defclass <getBlacklist-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass getBlacklist-response (<getBlacklist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getBlacklist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getBlacklist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getBlacklist-response> is deprecated: use state_estimator_msgs-srv:getBlacklist-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <getBlacklist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:names-val is deprecated.  Use state_estimator_msgs-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getBlacklist-response>) ostream)
  "Serializes a message object of type '<getBlacklist-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'names))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getBlacklist-response>) istream)
  "Deserializes a message object of type '<getBlacklist-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getBlacklist-response>)))
  "Returns string type for a service object of type '<getBlacklist-response>"
  "state_estimator_msgs/getBlacklistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getBlacklist-response)))
  "Returns string type for a service object of type 'getBlacklist-response"
  "state_estimator_msgs/getBlacklistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getBlacklist-response>)))
  "Returns md5sum for a message object of type '<getBlacklist-response>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getBlacklist-response)))
  "Returns md5sum for a message object of type 'getBlacklist-response"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getBlacklist-response>)))
  "Returns full string definition for message of type '<getBlacklist-response>"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getBlacklist-response)))
  "Returns full string definition for message of type 'getBlacklist-response"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getBlacklist-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getBlacklist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getBlacklist-response
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getBlacklist)))
  'getBlacklist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getBlacklist)))
  'getBlacklist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getBlacklist)))
  "Returns string type for a service object of type '<getBlacklist>"
  "state_estimator_msgs/getBlacklist")