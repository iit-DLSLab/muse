// Auto-generated. Do not edit!

// (in-package series_elastic_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SeActuatorCommand = require('./SeActuatorCommand.js');

//-----------------------------------------------------------

class SeActuatorCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.commands = null;
    }
    else {
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeActuatorCommands
    // Serialize message field [commands]
    // Serialize the length for message field [commands]
    bufferOffset = _serializer.uint32(obj.commands.length, buffer, bufferOffset);
    obj.commands.forEach((val) => {
      bufferOffset = SeActuatorCommand.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeActuatorCommands
    let len;
    let data = new SeActuatorCommands(null);
    // Deserialize message field [commands]
    // Deserialize array length for message field [commands]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.commands = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.commands[i] = SeActuatorCommand.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.commands.forEach((val) => {
      length += SeActuatorCommand.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'series_elastic_actuator_msgs/SeActuatorCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8bac105afcdb07f69773e72515936600';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Series-elastic actuator commands
    
    SeActuatorCommand[] commands
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SeActuatorCommands(null);
    if (msg.commands !== undefined) {
      resolved.commands = new Array(msg.commands.length);
      for (let i = 0; i < resolved.commands.length; ++i) {
        resolved.commands[i] = SeActuatorCommand.Resolve(msg.commands[i]);
      }
    }
    else {
      resolved.commands = []
    }

    return resolved;
    }
};

module.exports = SeActuatorCommands;
