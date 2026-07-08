; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude listAllEstimators-request.msg.html

(cl:defclass <listAllEstimators-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass listAllEstimators-request (<listAllEstimators-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <listAllEstimators-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'listAllEstimators-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<listAllEstimators-request> is deprecated: use state_estimator_msgs-srv:listAllEstimators-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <listAllEstimators-request>) ostream)
  "Serializes a message object of type '<listAllEstimators-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <listAllEstimators-request>) istream)
  "Deserializes a message object of type '<listAllEstimators-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<listAllEstimators-request>)))
  "Returns string type for a service object of type '<listAllEstimators-request>"
  "state_estimator_msgs/listAllEstimatorsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'listAllEstimators-request)))
  "Returns string type for a service object of type 'listAllEstimators-request"
  "state_estimator_msgs/listAllEstimatorsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<listAllEstimators-request>)))
  "Returns md5sum for a message object of type '<listAllEstimators-request>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'listAllEstimators-request)))
  "Returns md5sum for a message object of type 'listAllEstimators-request"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<listAllEstimators-request>)))
  "Returns full string definition for message of type '<listAllEstimators-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'listAllEstimators-request)))
  "Returns full string definition for message of type 'listAllEstimators-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <listAllEstimators-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <listAllEstimators-request>))
  "Converts a ROS message object to a list"
  (cl:list 'listAllEstimators-request
))
;//! \htmlinclude listAllEstimators-response.msg.html

(cl:defclass <listAllEstimators-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass listAllEstimators-response (<listAllEstimators-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <listAllEstimators-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'listAllEstimators-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<listAllEstimators-response> is deprecated: use state_estimator_msgs-srv:listAllEstimators-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <listAllEstimators-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:names-val is deprecated.  Use state_estimator_msgs-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <listAllEstimators-response>) ostream)
  "Serializes a message object of type '<listAllEstimators-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <listAllEstimators-response>) istream)
  "Deserializes a message object of type '<listAllEstimators-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<listAllEstimators-response>)))
  "Returns string type for a service object of type '<listAllEstimators-response>"
  "state_estimator_msgs/listAllEstimatorsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'listAllEstimators-response)))
  "Returns string type for a service object of type 'listAllEstimators-response"
  "state_estimator_msgs/listAllEstimatorsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<listAllEstimators-response>)))
  "Returns md5sum for a message object of type '<listAllEstimators-response>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'listAllEstimators-response)))
  "Returns md5sum for a message object of type 'listAllEstimators-response"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<listAllEstimators-response>)))
  "Returns full string definition for message of type '<listAllEstimators-response>"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'listAllEstimators-response)))
  "Returns full string definition for message of type 'listAllEstimators-response"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <listAllEstimators-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <listAllEstimators-response>))
  "Converts a ROS message object to a list"
  (cl:list 'listAllEstimators-response
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'listAllEstimators)))
  'listAllEstimators-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'listAllEstimators)))
  'listAllEstimators-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'listAllEstimators)))
  "Returns string type for a service object of type '<listAllEstimators>"
  "state_estimator_msgs/listAllEstimators")