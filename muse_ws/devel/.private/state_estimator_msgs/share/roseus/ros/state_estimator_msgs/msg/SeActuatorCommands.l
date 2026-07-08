;; Auto-generated. Do not edit!


(when (boundp 'state_estimator_msgs::SeActuatorCommands)
  (if (not (find-package "STATE_ESTIMATOR_MSGS"))
    (make-package "STATE_ESTIMATOR_MSGS"))
  (shadow 'SeActuatorCommands (find-package "STATE_ESTIMATOR_MSGS")))
(unless (find-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMANDS")
  (make-package "STATE_ESTIMATOR_MSGS::SEACTUATORCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude SeActuatorCommands.msg.html


(defclass state_estimator_msgs::SeActuatorCommands
  :super ros::object
  :slots (_commands ))

(defmethod state_estimator_msgs::SeActuatorCommands
  (:init
   (&key
    ((:commands __commands) ())
    )
   (send-super :init)
   (setq _commands __commands)
   self)
  (:commands
   (&rest __commands)
   (if (keywordp (car __commands))
       (send* _commands __commands)
     (progn
       (if __commands (setq _commands (car __commands)))
       _commands)))
  (:serialization-length
   ()
   (+
    ;; state_estimator_msgs/SeActuatorCommand[] _commands
    (apply #'+ (send-all _commands :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; state_estimator_msgs/SeActuatorCommand[] _commands
     (write-long (length _commands) s)
     (dolist (elem _commands)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; state_estimator_msgs/SeActuatorCommand[] _commands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _commands (let (r) (dotimes (i n) (push (instance state_estimator_msgs::SeActuatorCommand :init) r)) r))
     (dolist (elem- _commands)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get state_estimator_msgs::SeActuatorCommands :md5sum-) "8bac105afcdb07f69773e72515936600")
(setf (get state_estimator_msgs::SeActuatorCommands :datatype-) "state_estimator_msgs/SeActuatorCommands")
(setf (get state_estimator_msgs::SeActuatorCommands :definition-)
      "# Series-elastic actuator commands

SeActuatorCommand[] commands

================================================================================
MSG: state_estimator_msgs/SeActuatorCommand
# Series-elastic actuator command

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



(provide :state_estimator_msgs/SeActuatorCommands "8bac105afcdb07f69773e72515936600")


