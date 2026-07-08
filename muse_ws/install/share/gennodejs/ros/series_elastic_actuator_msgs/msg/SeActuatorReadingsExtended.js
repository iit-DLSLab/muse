// Auto-generated. Do not edit!

// (in-package series_elastic_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SeActuatorReadingExtended = require('./SeActuatorReadingExtended.js');

//-----------------------------------------------------------

class SeActuatorReadingsExtended {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.readings = null;
    }
    else {
      if (initObj.hasOwnProperty('readings')) {
        this.readings = initObj.readings
      }
      else {
        this.readings = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeActuatorReadingsExtended
    // Serialize message field [readings]
    // Serialize the length for message field [readings]
    bufferOffset = _serializer.uint32(obj.readings.length, buffer, bufferOffset);
    obj.readings.forEach((val) => {
      bufferOffset = SeActuatorReadingExtended.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeActuatorReadingsExtended
    let len;
    let data = new SeActuatorReadingsExtended(null);
    // Deserialize message field [readings]
    // Deserialize array length for message field [readings]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.readings = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.readings[i] = SeActuatorReadingExtended.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.readings.forEach((val) => {
      length += SeActuatorReadingExtended.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'series_elastic_actuator_msgs/SeActuatorReadingsExtended';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a8f2c5a9ba71ba57ebe8e54f9d205c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Series-elastic actuator readings extended
    
    SeActuatorReadingExtended[] readings
    
    ================================================================================
    MSG: series_elastic_actuator_msgs/SeActuatorReadingExtended
    # Series-elastic actuator extended reading
    #
    Header header
    # States
    SeActuatorStateExtended state
    # Set-points
    SeActuatorCommand commanded
    
    
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
    MSG: series_elastic_actuator_msgs/SeActuatorStateExtended
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
    
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
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
    MSG: series_elastic_actuator_msgs/SeActuatorCommand
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SeActuatorReadingsExtended(null);
    if (msg.readings !== undefined) {
      resolved.readings = new Array(msg.readings.length);
      for (let i = 0; i < resolved.readings.length; ++i) {
        resolved.readings[i] = SeActuatorReadingExtended.Resolve(msg.readings[i]);
      }
    }
    else {
      resolved.readings = []
    }

    return resolved;
    }
};

module.exports = SeActuatorReadingsExtended;
