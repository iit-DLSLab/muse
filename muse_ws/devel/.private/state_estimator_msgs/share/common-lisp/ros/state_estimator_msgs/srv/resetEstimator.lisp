; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude resetEstimator-request.msg.html

(cl:defclass <resetEstimator-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass resetEstimator-request (<resetEstimator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resetEstimator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resetEstimator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<resetEstimator-request> is deprecated: use state_estimator_msgs-srv:resetEstimator-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <resetEstimator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:name-val is deprecated.  Use state_estimator_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resetEstimator-request>) ostream)
  "Serializes a message object of type '<resetEstimator-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resetEstimator-request>) istream)
  "Deserializes a message object of type '<resetEstimator-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resetEstimator-request>)))
  "Returns string type for a service object of type '<resetEstimator-request>"
  "state_estimator_msgs/resetEstimatorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resetEstimator-request)))
  "Returns string type for a service object of type 'resetEstimator-request"
  "state_estimator_msgs/resetEstimatorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resetEstimator-request>)))
  "Returns md5sum for a message object of type '<resetEstimator-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resetEstimator-request)))
  "Returns md5sum for a message object of type 'resetEstimator-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resetEstimator-request>)))
  "Returns full string definition for message of type '<resetEstimator-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resetEstimator-request)))
  "Returns full string definition for message of type 'resetEstimator-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resetEstimator-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resetEstimator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'resetEstimator-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude resetEstimator-response.msg.html

(cl:defclass <resetEstimator-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass resetEstimator-response (<resetEstimator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resetEstimator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resetEstimator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<resetEstimator-response> is deprecated: use state_estimator_msgs-srv:resetEstimator-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <resetEstimator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:success-val is deprecated.  Use state_estimator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resetEstimator-response>) ostream)
  "Serializes a message object of type '<resetEstimator-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resetEstimator-response>) istream)
  "Deserializes a message object of type '<resetEstimator-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resetEstimator-response>)))
  "Returns string type for a service object of type '<resetEstimator-response>"
  "state_estimator_msgs/resetEstimatorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resetEstimator-response)))
  "Returns string type for a service object of type 'resetEstimator-response"
  "state_estimator_msgs/resetEstimatorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resetEstimator-response>)))
  "Returns md5sum for a message object of type '<resetEstimator-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resetEstimator-response)))
  "Returns md5sum for a message object of type 'resetEstimator-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resetEstimator-response>)))
  "Returns full string definition for message of type '<resetEstimator-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resetEstimator-response)))
  "Returns full string definition for message of type 'resetEstimator-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resetEstimator-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resetEstimator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'resetEstimator-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'resetEstimator)))
  'resetEstimator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'resetEstimator)))
  'resetEstimator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resetEstimator)))
  "Returns string type for a service object of type '<resetEstimator>"
  "state_estimator_msgs/resetEstimator")