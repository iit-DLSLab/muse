;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::SeActuatorCommand)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'SeActuatorCommand (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND")
  (make-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SeActuatorCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MODE_NA*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_NA* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_NA* 0)
(intern "*MODE_FREEZE*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_FREEZE* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_FREEZE* 1)
(intern "*MODE_DISABLE*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_DISABLE* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_DISABLE* 2)
(intern "*MODE_CURRENT*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_CURRENT* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_CURRENT* 3)
(intern "*MODE_MOTOR_POSITION*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_MOTOR_POSITION* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_MOTOR_POSITION* 4)
(intern "*MODE_MOTOR_VELOCITY*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_MOTOR_VELOCITY* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_MOTOR_VELOCITY* 5)
(intern "*MODE_GEAR_POSITION*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_GEAR_POSITION* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_GEAR_POSITION* 6)
(intern "*MODE_GEAR_VELOCITY*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_GEAR_VELOCITY* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_GEAR_VELOCITY* 7)
(intern "*MODE_JOINT_POSITION*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_POSITION* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION* 8)
(intern "*MODE_JOINT_VELOCITY*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_VELOCITY* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_VELOCITY* 9)
(intern "*MODE_JOINT_TORQUE*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_TORQUE* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_TORQUE* 10)
(intern "*MODE_JOINT_POSITION_VELOCITY*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_POSITION_VELOCITY* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY* 11)
(intern "*MODE_JOINT_POSITION_VELOCITY_TORQUE*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_POSITION_VELOCITY_TORQUE* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY_TORQUE* 12)
(intern "*MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS*" (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(shadow '*MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS* (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMAND"))
(defconstant state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS* 13)

(defun state_estimator_msgs::SeActuatorCommand-to-symbol (const)
  (cond
        ((= const 0) 'state_estimator_msgs::SeActuatorCommand::*MODE_NA*)
        ((= const 1) 'state_estimator_msgs::SeActuatorCommand::*MODE_FREEZE*)
        ((= const 2) 'state_estimator_msgs::SeActuatorCommand::*MODE_DISABLE*)
        ((= const 3) 'state_estimator_msgs::SeActuatorCommand::*MODE_CURRENT*)
        ((= const 4) 'state_estimator_msgs::SeActuatorCommand::*MODE_MOTOR_POSITION*)
        ((= const 5) 'state_estimator_msgs::SeActuatorCommand::*MODE_MOTOR_VELOCITY*)
        ((= const 6) 'state_estimator_msgs::SeActuatorCommand::*MODE_GEAR_POSITION*)
        ((= const 7) 'state_estimator_msgs::SeActuatorCommand::*MODE_GEAR_VELOCITY*)
        ((= const 8) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION*)
        ((= const 9) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_VELOCITY*)
        ((= const 10) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_TORQUE*)
        ((= const 11) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY*)
        ((= const 12) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY_TORQUE*)
        ((= const 13) 'state_estimator_msgs::SeActuatorCommand::*MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS*)
        (t nil)))

(defclass state_estimator_msgs::SeActuatorCommand
  :super ros::object
  :slots (_header _name _mode _current _position _velocity _joint_torque _pid_gains_p _pid_gains_i _pid_gains_d ))

(defmethod state_estimator_msgs::SeActuatorCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:mode __mode) 0)
    ((:current __current) 0.0)
    ((:position __position) 0.0)
    ((:velocity __velocity) 0.0)
    ((:joint_torque __joint_torque) 0.0)
    ((:pid_gains_p __pid_gains_p) 0.0)
    ((:pid_gains_i __pid_gains_i) 0.0)
    ((:pid_gains_d __pid_gains_d) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _mode (round __mode))
   (setq _current (float __current))
   (setq _position (float __position))
   (setq _velocity (float __velocity))
   (setq _joint_torque (float __joint_torque))
   (setq _pid_gains_p (float __pid_gains_p))
   (setq _pid_gains_i (float __pid_gains_i))
   (setq _pid_gains_d (float __pid_gains_d))
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:joint_torque
   (&optional __joint_torque)
   (if __joint_torque (setq _joint_torque __joint_torque)) _joint_torque)
  (:pid_gains_p
   (&optional __pid_gains_p)
   (if __pid_gains_p (setq _pid_gains_p __pid_gains_p)) _pid_gains_p)
  (:pid_gains_i
   (&optional __pid_gains_i)
   (if __pid_gains_i (setq _pid_gains_i __pid_gains_i)) _pid_gains_i)
  (:pid_gains_d
   (&optional __pid_gains_d)
   (if __pid_gains_d (setq _pid_gains_d __pid_gains_d)) _pid_gains_d)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; int16 _mode
    2
    ;; float64 _current
    8
    ;; float64 _position
    8
    ;; float64 _velocity
    8
    ;; float64 _joint_torque
    8
    ;; float32 _pid_gains_p
    4
    ;; float32 _pid_gains_i
    4
    ;; float32 _pid_gains_d
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int16 _mode
       (write-word _mode s)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint_torque
       (sys::poke _joint_torque (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _pid_gains_p
       (sys::poke _pid_gains_p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pid_gains_i
       (sys::poke _pid_gains_i (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pid_gains_d
       (sys::poke _pid_gains_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _mode
     (setq _mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _velocity
     (setq _velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint_torque
     (setq _joint_torque (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _pid_gains_p
     (setq _pid_gains_p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pid_gains_i
     (setq _pid_gains_i (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pid_gains_d
     (setq _pid_gains_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get state_estimator_msgs::SeActuatorCommand :md5sum-) "99caa130fb942b20d4b16f18464e90d2")
(setf (get state_estimator_msgs::SeActuatorCommand :datatype-) "state_estimator_msgs/SeActuatorCommand")
(setf (get state_estimator_msgs::SeActuatorCommand :definition-)
      "# Series-elastic actuator command

Header header
# Name of the actuator
string name
# Control mode
int16 MODE_NA                                       = 0  # Not available
int16 MODE_FREEZE                                   = 1  # Freeze motor
int16 MODE_DISABLE                                  = 2  # Disable motor
int16 MODE_CURRENT                                  = 3  # Track current
int16 MODE_MOTOR_POSITION                           = 4  # Track motor position
int16 MODE_MOTOR_VELOCITY                           = 5  # Track motor velocity
int16 MODE_GEAR_POSITION                            = 6  # Track gear position
int16 MODE_GEAR_VELOCITY                            = 7  # Track gear velocity
int16 MODE_JOINT_POSITION                           = 8  # Track joint position
int16 MODE_JOINT_VELOCITY                           = 9  # Track joint velocity
int16 MODE_JOINT_TORQUE                             = 10 # Track joint torque
int16 MODE_JOINT_POSITION_VELOCITY                  = 11 # Track joint position with feedforward velocity
int16 MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12 # Track joint position with feedforward velocity and torque
int16 MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13 # Track joint position with feedforward velocity and torque using custom joint position gains
int16 mode
# Motor current [A]
float64 current
# Position (motor/gear/joint) [rad]
float64 position
# Velocity (motor/gear/joint) [rad/s]
float64 velocity
# Torque (joint) [Nm]
float64 joint_torque
# Custom PID gains [-]
float32 pid_gains_p
float32 pid_gains_i
float32 pid_gains_d

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

")



(provide :state_estimator_msgs/SeActuatorCommand "99caa130fb942b20d4b16f18464e90d2")


