;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::AnymalState)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'AnymalState (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE")
  (make-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))

(in-package "ROS")
;;//! \htmlinclude AnymalState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_ERROR_SENSOR*" (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(shadow '*STATE_ERROR_SENSOR* (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(defconstant state_estimator_msgs::AnymalState::*STATE_ERROR_SENSOR* -3)
(intern "*STATE_ERROR_ESTIMATOR*" (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(shadow '*STATE_ERROR_ESTIMATOR* (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(defconstant state_estimator_msgs::AnymalState::*STATE_ERROR_ESTIMATOR* -2)
(intern "*STATE_ERROR_UNKNOWN*" (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(shadow '*STATE_ERROR_UNKNOWN* (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(defconstant state_estimator_msgs::AnymalState::*STATE_ERROR_UNKNOWN* -1)
(intern "*STATE_OK*" (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(shadow '*STATE_OK* (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(defconstant state_estimator_msgs::AnymalState::*STATE_OK* 0)
(intern "*STATE_UNINITIALIZED*" (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(shadow '*STATE_UNINITIALIZED* (find-package "STATE_ESTIMATOR_MSGS::ANYMALSTATE"))
(defconstant state_estimator_msgs::AnymalState::*STATE_UNINITIALIZED* 1)

(defun state_estimator_msgs::AnymalState-to-symbol (const)
  (cond
        ((= const -3) 'state_estimator_msgs::AnymalState::*STATE_ERROR_SENSOR*)
        ((= const -2) 'state_estimator_msgs::AnymalState::*STATE_ERROR_ESTIMATOR*)
        ((= const -1) 'state_estimator_msgs::AnymalState::*STATE_ERROR_UNKNOWN*)
        ((= const 0) 'state_estimator_msgs::AnymalState::*STATE_OK*)
        ((= const 1) 'state_estimator_msgs::AnymalState::*STATE_UNINITIALIZED*)
        (t nil)))

(defclass state_estimator_msgs::AnymalState
  :super ros::object
  :slots (_header _state _pose _twist _joints _contacts _frame_transforms ))

(defmethod state_estimator_msgs::AnymalState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:twist __twist) (instance geometry_msgs::TwistStamped :init))
    ((:joints __joints) (instance state_estimator_msgs::ExtendedJointState :init))
    ((:contacts __contacts) ())
    ((:frame_transforms __frame_transforms) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _joints __joints)
   (setq _contacts __contacts)
   (setq _frame_transforms __frame_transforms)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:contacts
   (&rest __contacts)
   (if (keywordp (car __contacts))
       (send* _contacts __contacts)
     (progn
       (if __contacts (setq _contacts (car __contacts)))
       _contacts)))
  (:frame_transforms
   (&rest __frame_transforms)
   (if (keywordp (car __frame_transforms))
       (send* _frame_transforms __frame_transforms)
     (progn
       (if __frame_transforms (setq _frame_transforms (car __frame_transforms)))
       _frame_transforms)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _state
    1
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/TwistStamped _twist
    (send _twist :serialization-length)
    ;; state_estimator_msgs/ExtendedJointState _joints
    (send _joints :serialization-length)
    ;; state_estimator_msgs/Contact[] _contacts
    (apply #'+ (send-all _contacts :serialization-length)) 4
    ;; geometry_msgs/TransformStamped[] _frame_transforms
    (apply #'+ (send-all _frame_transforms :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _state
       (write-byte _state s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; geometry_msgs/TwistStamped _twist
       (send _twist :serialize s)
     ;; state_estimator_msgs/ExtendedJointState _joints
       (send _joints :serialize s)
     ;; state_estimator_msgs/Contact[] _contacts
     (write-long (length _contacts) s)
     (dolist (elem _contacts)
       (send elem :serialize s)
       )
     ;; geometry_msgs/TransformStamped[] _frame_transforms
     (write-long (length _frame_transforms) s)
     (dolist (elem _frame_transforms)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _state 127) (setq _state (- _state 256)))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/TwistStamped _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; state_estimator_msgs/ExtendedJointState _joints
     (send _joints :deserialize buf ptr-) (incf ptr- (send _joints :serialization-length))
   ;; state_estimator_msgs/Contact[] _contacts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contacts (let (r) (dotimes (i n) (push (instance state_estimator_msgs::Contact :init) r)) r))
     (dolist (elem- _contacts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/TransformStamped[] _frame_transforms
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frame_transforms (let (r) (dotimes (i n) (push (instance geometry_msgs::TransformStamped :init) r)) r))
     (dolist (elem- _frame_transforms)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get state_estimator_msgs::AnymalState :md5sum-) "1bee66925f9d04663ad1cd7f4a496f93")
(setf (get state_estimator_msgs::AnymalState :datatype-) "state_estimator_msgs/AnymalState")
(setf (get state_estimator_msgs::AnymalState :definition-)
      "# Anymal state including generalized coordinates, velocities, and contacts.
std_msgs/Header header

int8 STATE_ERROR_SENSOR=-3
int8 STATE_ERROR_ESTIMATOR=-2
int8 STATE_ERROR_UNKNOWN=-1
int8 STATE_OK=0
int8 STATE_UNINITIALIZED=1
int8 state

# Position of the base with respect to the odom frame, and orientation from base to odom.
geometry_msgs/PoseStamped pose

# Linear velocity and local angular velocity expressed in base frame.
geometry_msgs/TwistStamped twist
ExtendedJointState joints
Contact[] contacts
geometry_msgs/TransformStamped[] frame_transforms

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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
MSG: state_estimator_msgs/ExtendedJointState
# sensor_msgs/JointState with an additional field for acceleration.
std_msgs/Header header

string[] name
float64[] position
float64[] velocity
float64[] acceleration
float64[] effort

================================================================================
MSG: state_estimator_msgs/Contact
# Contact definition.
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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

")



(provide :state_estimator_msgs/AnymalState "1bee66925f9d04663ad1cd7f4a496f93")


