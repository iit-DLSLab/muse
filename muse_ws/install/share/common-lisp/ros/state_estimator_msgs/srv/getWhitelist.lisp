; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude getWhitelist-request.msg.html

(cl:defclass <getWhitelist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getWhitelist-request (<getWhitelist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getWhitelist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getWhitelist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getWhitelist-request> is deprecated: use state_estimator_msgs-srv:getWhitelist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getWhitelist-request>) ostream)
  "Serializes a message object of type '<getWhitelist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getWhitelist-request>) istream)
  "Deserializes a message object of type '<getWhitelist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getWhitelist-request>)))
  "Returns string type for a service object of type '<getWhitelist-request>"
  "state_estimator_msgs/getWhitelistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWhitelist-request)))
  "Returns string type for a service object of type 'getWhitelist-request"
  "state_estimator_msgs/getWhitelistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getWhitelist-request>)))
  "Returns md5sum for a message object of type '<getWhitelist-request>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getWhitelist-request)))
  "Returns md5sum for a message object of type 'getWhitelist-request"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getWhitelist-request>)))
  "Returns full string definition for message of type '<getWhitelist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getWhitelist-request)))
  "Returns full string definition for message of type 'getWhitelist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getWhitelist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getWhitelist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getWhitelist-request
))
;//! \htmlinclude getWhitelist-response.msg.html

(cl:defclass <getWhitelist-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass getWhitelist-response (<getWhitelist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getWhitelist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getWhitelist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getWhitelist-response> is deprecated: use state_estimator_msgs-srv:getWhitelist-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <getWhitelist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:names-val is deprecated.  Use state_estimator_msgs-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getWhitelist-response>) ostream)
  "Serializes a message object of type '<getWhitelist-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getWhitelist-response>) istream)
  "Deserializes a message object of type '<getWhitelist-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getWhitelist-response>)))
  "Returns string type for a service object of type '<getWhitelist-response>"
  "state_estimator_msgs/getWhitelistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWhitelist-response)))
  "Returns string type for a service object of type 'getWhitelist-response"
  "state_estimator_msgs/getWhitelistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getWhitelist-response>)))
  "Returns md5sum for a message object of type '<getWhitelist-response>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getWhitelist-response)))
  "Returns md5sum for a message object of type 'getWhitelist-response"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getWhitelist-response>)))
  "Returns full string definition for message of type '<getWhitelist-response>"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getWhitelist-response)))
  "Returns full string definition for message of type 'getWhitelist-response"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getWhitelist-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getWhitelist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getWhitelist-response
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getWhitelist)))
  'getWhitelist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getWhitelist)))
  'getWhitelist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getWhitelist)))
  "Returns string type for a service object of type '<getWhitelist>"
  "state_estimator_msgs/getWhitelist")