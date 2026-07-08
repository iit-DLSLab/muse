; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude getActiveEstimators-request.msg.html

(cl:defclass <getActiveEstimators-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getActiveEstimators-request (<getActiveEstimators-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getActiveEstimators-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getActiveEstimators-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getActiveEstimators-request> is deprecated: use state_estimator_msgs-srv:getActiveEstimators-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getActiveEstimators-request>) ostream)
  "Serializes a message object of type '<getActiveEstimators-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getActiveEstimators-request>) istream)
  "Deserializes a message object of type '<getActiveEstimators-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getActiveEstimators-request>)))
  "Returns string type for a service object of type '<getActiveEstimators-request>"
  "state_estimator_msgs/getActiveEstimatorsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getActiveEstimators-request)))
  "Returns string type for a service object of type 'getActiveEstimators-request"
  "state_estimator_msgs/getActiveEstimatorsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getActiveEstimators-request>)))
  "Returns md5sum for a message object of type '<getActiveEstimators-request>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getActiveEstimators-request)))
  "Returns md5sum for a message object of type 'getActiveEstimators-request"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getActiveEstimators-request>)))
  "Returns full string definition for message of type '<getActiveEstimators-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getActiveEstimators-request)))
  "Returns full string definition for message of type 'getActiveEstimators-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getActiveEstimators-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getActiveEstimators-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getActiveEstimators-request
))
;//! \htmlinclude getActiveEstimators-response.msg.html

(cl:defclass <getActiveEstimators-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass getActiveEstimators-response (<getActiveEstimators-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getActiveEstimators-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getActiveEstimators-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getActiveEstimators-response> is deprecated: use state_estimator_msgs-srv:getActiveEstimators-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <getActiveEstimators-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:names-val is deprecated.  Use state_estimator_msgs-srv:names instead.")
  (names m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getActiveEstimators-response>) ostream)
  "Serializes a message object of type '<getActiveEstimators-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getActiveEstimators-response>) istream)
  "Deserializes a message object of type '<getActiveEstimators-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getActiveEstimators-response>)))
  "Returns string type for a service object of type '<getActiveEstimators-response>"
  "state_estimator_msgs/getActiveEstimatorsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getActiveEstimators-response)))
  "Returns string type for a service object of type 'getActiveEstimators-response"
  "state_estimator_msgs/getActiveEstimatorsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getActiveEstimators-response>)))
  "Returns md5sum for a message object of type '<getActiveEstimators-response>"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getActiveEstimators-response)))
  "Returns md5sum for a message object of type 'getActiveEstimators-response"
  "dc7ae3609524b18034e49294a4ce670e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getActiveEstimators-response>)))
  "Returns full string definition for message of type '<getActiveEstimators-response>"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getActiveEstimators-response)))
  "Returns full string definition for message of type 'getActiveEstimators-response"
  (cl:format cl:nil "string[] names~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getActiveEstimators-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getActiveEstimators-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getActiveEstimators-response
    (cl:cons ':names (names msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getActiveEstimators)))
  'getActiveEstimators-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getActiveEstimators)))
  'getActiveEstimators-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getActiveEstimators)))
  "Returns string type for a service object of type '<getActiveEstimators>"
  "state_estimator_msgs/getActiveEstimators")