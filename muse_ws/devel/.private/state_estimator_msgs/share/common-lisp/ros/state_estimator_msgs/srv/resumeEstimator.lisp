; Auto-generated. Do not edit!


(cl:in-package state_estimator_msgs-srv)


;//! \htmlinclude resumeEstimator-request.msg.html

(cl:defclass <resumeEstimator-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass resumeEstimator-request (<resumeEstimator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resumeEstimator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resumeEstimator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<resumeEstimator-request> is deprecated: use state_estimator_msgs-srv:resumeEstimator-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <resumeEstimator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:name-val is deprecated.  Use state_estimator_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resumeEstimator-request>) ostream)
  "Serializes a message object of type '<resumeEstimator-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resumeEstimator-request>) istream)
  "Deserializes a message object of type '<resumeEstimator-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resumeEstimator-request>)))
  "Returns string type for a service object of type '<resumeEstimator-request>"
  "state_estimator_msgs/resumeEstimatorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resumeEstimator-request)))
  "Returns string type for a service object of type 'resumeEstimator-request"
  "state_estimator_msgs/resumeEstimatorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resumeEstimator-request>)))
  "Returns md5sum for a message object of type '<resumeEstimator-request>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resumeEstimator-request)))
  "Returns md5sum for a message object of type 'resumeEstimator-request"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resumeEstimator-request>)))
  "Returns full string definition for message of type '<resumeEstimator-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resumeEstimator-request)))
  "Returns full string definition for message of type 'resumeEstimator-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resumeEstimator-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resumeEstimator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'resumeEstimator-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude resumeEstimator-response.msg.html

(cl:defclass <resumeEstimator-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass resumeEstimator-response (<resumeEstimator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resumeEstimator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resumeEstimator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name state_estimator_msgs-srv:<resumeEstimator-response> is deprecated: use state_estimator_msgs-srv:resumeEstimator-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <resumeEstimator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader state_estimator_msgs-srv:success-val is deprecated.  Use state_estimator_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resumeEstimator-response>) ostream)
  "Serializes a message object of type '<resumeEstimator-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resumeEstimator-response>) istream)
  "Deserializes a message object of type '<resumeEstimator-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resumeEstimator-response>)))
  "Returns string type for a service object of type '<resumeEstimator-response>"
  "state_estimator_msgs/resumeEstimatorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resumeEstimator-response)))
  "Returns string type for a service object of type 'resumeEstimator-response"
  "state_estimator_msgs/resumeEstimatorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resumeEstimator-response>)))
  "Returns md5sum for a message object of type '<resumeEstimator-response>"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resumeEstimator-response)))
  "Returns md5sum for a message object of type 'resumeEstimator-response"
  "d08a3b641c2f8680fbdfb1ea2e17a3e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resumeEstimator-response>)))
  "Returns full string definition for message of type '<resumeEstimator-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resumeEstimator-response)))
  "Returns full string definition for message of type 'resumeEstimator-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resumeEstimator-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resumeEstimator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'resumeEstimator-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'resumeEstimator)))
  'resumeEstimator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'resumeEstimator)))
  'resumeEstimator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resumeEstimator)))
  "Returns string type for a service object of type '<resumeEstimator>"
  "state_estimator_msgs/resumeEstimator")