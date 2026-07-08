; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude getEstimatorDescription-request.msg.html

(cl:defclass <getEstimatorDescription-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass getEstimatorDescription-request (<getEstimatorDescription-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getEstimatorDescription-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getEstimatorDescription-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getEstimatorDescription-request> is deprecated: use state_estimator_msgs-srv:getEstimatorDescription-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <getEstimatorDescription-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:name-val is deprecated.  Use state_estimator_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getEstimatorDescription-request>) ostream)
  "Serializes a message object of type '<getEstimatorDescription-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getEstimatorDescription-request>) istream)
  "Deserializes a message object of type '<getEstimatorDescription-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getEstimatorDescription-request>)))
  "Returns string type for a service object of type '<getEstimatorDescription-request>"
  "state_estimator_msgs/getEstimatorDescriptionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getEstimatorDescription-request)))
  "Returns string type for a service object of type 'getEstimatorDescription-request"
  "state_estimator_msgs/getEstimatorDescriptionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getEstimatorDescription-request>)))
  "Returns md5sum for a message object of type '<getEstimatorDescription-request>"
  "12c3019909e7fa7b3482cdf9594556c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getEstimatorDescription-request)))
  "Returns md5sum for a message object of type 'getEstimatorDescription-request"
  "12c3019909e7fa7b3482cdf9594556c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getEstimatorDescription-request>)))
  "Returns full string definition for message of type '<getEstimatorDescription-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getEstimatorDescription-request)))
  "Returns full string definition for message of type 'getEstimatorDescription-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getEstimatorDescription-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getEstimatorDescription-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getEstimatorDescription-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude getEstimatorDescription-response.msg.html

(cl:defclass <getEstimatorDescription-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass getEstimatorDescription-response (<getEstimatorDescription-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getEstimatorDescription-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getEstimatorDescription-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<getEstimatorDescription-response> is deprecated: use state_estimator_msgs-srv:getEstimatorDescription-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <getEstimatorDescription-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:success-val is deprecated.  Use state_estimator_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <getEstimatorDescription-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:description-val is deprecated.  Use state_estimator_msgs-srv:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getEstimatorDescription-response>) ostream)
  "Serializes a message object of type '<getEstimatorDescription-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getEstimatorDescription-response>) istream)
  "Deserializes a message object of type '<getEstimatorDescription-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getEstimatorDescription-response>)))
  "Returns string type for a service object of type '<getEstimatorDescription-response>"
  "state_estimator_msgs/getEstimatorDescriptionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getEstimatorDescription-response)))
  "Returns string type for a service object of type 'getEstimatorDescription-response"
  "state_estimator_msgs/getEstimatorDescriptionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getEstimatorDescription-response>)))
  "Returns md5sum for a message object of type '<getEstimatorDescription-response>"
  "12c3019909e7fa7b3482cdf9594556c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getEstimatorDescription-response)))
  "Returns md5sum for a message object of type 'getEstimatorDescription-response"
  "12c3019909e7fa7b3482cdf9594556c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getEstimatorDescription-response>)))
  "Returns full string definition for message of type '<getEstimatorDescription-response>"
  (cl:format cl:nil "bool success~%string description~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getEstimatorDescription-response)))
  "Returns full string definition for message of type 'getEstimatorDescription-response"
  (cl:format cl:nil "bool success~%string description~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getEstimatorDescription-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getEstimatorDescription-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getEstimatorDescription-response
    (cl:cons ':success (success msg))
    (cl:cons ':description (description msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getEstimatorDescription)))
  'getEstimatorDescription-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getEstimatorDescription)))
  'getEstimatorDescription-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getEstimatorDescription)))
  "Returns string type for a service object of type '<getEstimatorDescription>"
  "state_estimator_msgs/getEstimatorDescription")