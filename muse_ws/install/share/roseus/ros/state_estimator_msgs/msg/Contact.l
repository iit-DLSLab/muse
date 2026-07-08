;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::Contact)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'Contact (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::CONTACT")
  (make-package "STATE_ESTIMATOR_MSGS::CONTACT"))

(in-package "ROS")
;;//! \htmlinclude Contact.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_OPEN*" (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(shadow '*STATE_OPEN* (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(defconstant state_estimator_msgs::Contact::*STATE_OPEN* 0)
(intern "*STATE_CLOSED*" (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(shadow '*STATE_CLOSED* (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(defconstant state_estimator_msgs::Contact::*STATE_CLOSED* 1)
(intern "*STATE_SLIPPING*" (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(shadow '*STATE_SLIPPING* (find-package "STATE_ESTIMATOR_MSGS::CONTACT"))
(defconstant state_estimator_msgs::Contact::*STATE_SLIPPING* 2)

(defun state_estimator_msgs::Contact-to-symbol (const)
  (cond
        ((= const 0) 'state_estimator_msgs::Contact::*STATE_OPEN*)
        ((= const 1) 'state_estimator_msgs::Contact::*STATE_CLOSED*)
        ((= const 2) 'state_estimator_msgs::Contact::*STATE_SLIPPING*)
        (t nil)))

(defclass state_estimator_msgs::Contact
  :super ros::object
  :slots (_header _name _state _wrench _position _normal _frictionCoefficient _restitutionCoefficient ))

(defmethod state_estimator_msgs::Contact
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:state __state) 0)
    ((:wrench __wrench) (instance geometry_msgs::Wrench :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:normal __normal) (instance geometry_msgs::Vector3 :init))
    ((:frictionCoefficient __frictionCoefficient) 0.0)
    ((:restitutionCoefficient __restitutionCoefficient) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _state (round __state))
   (setq _wrench __wrench)
   (setq _position __position)
   (setq _normal __normal)
   (setq _frictionCoefficient (float __frictionCoefficient))
   (setq _restitutionCoefficient (float __restitutionCoefficient))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:normal
   (&rest __normal)
   (if (keywordp (car __normal))
       (send* _normal __normal)
     (progn
       (if __normal (setq _normal (car __normal)))
       _normal)))
  (:frictionCoefficient
   (&optional __frictionCoefficient)
   (if __frictionCoefficient (setq _frictionCoefficient __frictionCoefficient)) _frictionCoefficient)
  (:restitutionCoefficient
   (&optional __restitutionCoefficient)
   (if __restitutionCoefficient (setq _restitutionCoefficient __restitutionCoefficient)) _restitutionCoefficient)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint8 _state
    1
    ;; geometry_msgs/Wrench _wrench
    (send _wrench :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _normal
    (send _normal :serialization-length)
    ;; float64 _frictionCoefficient
    8
    ;; float64 _restitutionCoefficient
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint8 _state
       (write-byte _state s)
     ;; geometry_msgs/Wrench _wrench
       (send _wrench :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _normal
       (send _normal :serialize s)
     ;; float64 _frictionCoefficient
       (sys::poke _frictionCoefficient (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _restitutionCoefficient
       (sys::poke _restitutionCoefficient (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _normal
     (send _normal :deserialize buf ptr-) (incf ptr- (send _normal :serialization-length))
   ;; float64 _frictionCoefficient
     (setq _frictionCoefficient (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _restitutionCoefficient
     (setq _restitutionCoefficient (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get state_estimator_msgs::Contact :md5sum-) "48c55012030000155d5ac40f7586c2e6")
(setf (get state_estimator_msgs::Contact :datatype-) "state_estimator_msgs/Contact")
(setf (get state_estimator_msgs::Contact :definition-)
      "# Contact definition.
uint8 STATE_OPEN=0
uint8 STATE_CLOSED=1
uint8 STATE_SLIPPING=2

std_msgs/Header header
string name
uint8 state
geometry_msgs/Wrench wrench
geometry_msgs/Point position
geometry_msgs/Vector3 normal
float64 frictionCoefficient
float64 restitutionCoefficient

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :state_estimator_msgs/Contact "48c55012030000155d5ac40f7586c2e6")


