# Series-elastic actuator extended state

# Header of the message
Header header
# Name of the actuator
string name
# Statusword
uint32 statusword
# Motor current [A]
float64 current
# Position of the spring on the gear side [rad]
float64 gear_position
# Velocity of the spring on the gear side [rad/s]
float64 gear_velocity
# Position of the spring on the joint side [rad]
float64 joint_position
# Velocity of the spring on the joint side [rad/s]
float64 joint_velocity
# Acceleration of the spring on the joint side [rad/s^2]
float64 joint_acceleration
# Joint torque [Nm]
float64 joint_torque
# IMU measurement
sensor_msgs/Imu imu

# Extended state data:

# Position of the motor [rad]
float64 motor_position
# Velocity of the motor [rad/s]
float64 motor_velocity
# Position of the spring on the gear side [ticks]
int32 gear_position_ticks
# Position of the spring on the joint side [ticks]
int32 joint_position_ticks
# Temperature of the actuator [deg C]
float64 temperature
# Voltage [V]
float64 voltage
# Timestamp
uint64 timestamp
# Desired current d part
float64 desired_current_d
# Measured current d part
float64 measured_current_d
# Desired current q part
float64 desired_current_q
# Measured current q part
float64 measured_current_q
# Measured current phase u
float64 measured_current_phase_u
# Measured voltage phase u
float64 measured_voltage_phase_u
# Measured current phase v
float64 measured_current_phase_v
# Measured voltage phase v
float64 measured_voltage_phase_v
# Measured current phase w
float64 measured_current_phase_w
# Measured voltage phase w
float64 measured_voltage_phase_w

