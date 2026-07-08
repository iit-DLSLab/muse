// Auto-generated. Do not edit!

// (in-package series_elastic_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SeActuatorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.mode = null;
      this.current = null;
      this.position = null;
      this.velocity = null;
      this.joint_torque = null;
      this.pid_gains_p = null;
      this.pid_gains_i = null;
      this.pid_gains_d = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('joint_torque')) {
        this.joint_torque = initObj.joint_torque
      }
      else {
        this.joint_torque = 0.0;
      }
      if (initObj.hasOwnProperty('pid_gains_p')) {
        this.pid_gains_p = initObj.pid_gains_p
      }
      else {
        this.pid_gains_p = 0.0;
      }
      if (initObj.hasOwnProperty('pid_gains_i')) {
        this.pid_gains_i = initObj.pid_gains_i
      }
      else {
        this.pid_gains_i = 0.0;
      }
      if (initObj.hasOwnProperty('pid_gains_d')) {
        this.pid_gains_d = initObj.pid_gains_d
      }
      else {
        this.pid_gains_d = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeActuatorCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int16(obj.mode, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [joint_torque]
    bufferOffset = _serializer.float64(obj.joint_torque, buffer, bufferOffset);
    // Serialize message field [pid_gains_p]
    bufferOffset = _serializer.float32(obj.pid_gains_p, buffer, bufferOffset);
    // Serialize message field [pid_gains_i]
    bufferOffset = _serializer.float32(obj.pid_gains_i, buffer, bufferOffset);
    // Serialize message field [pid_gains_d]
    bufferOffset = _serializer.float32(obj.pid_gains_d, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeActuatorCommand
    let len;
    let data = new SeActuatorCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_torque]
    data.joint_torque = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pid_gains_p]
    data.pid_gains_p = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pid_gains_i]
    data.pid_gains_i = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pid_gains_d]
    data.pid_gains_d = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.name);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'series_elastic_actuator_msgs/SeActuatorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99caa130fb942b20d4b16f18464e90d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SeActuatorCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.joint_torque !== undefined) {
      resolved.joint_torque = msg.joint_torque;
    }
    else {
      resolved.joint_torque = 0.0
    }

    if (msg.pid_gains_p !== undefined) {
      resolved.pid_gains_p = msg.pid_gains_p;
    }
    else {
      resolved.pid_gains_p = 0.0
    }

    if (msg.pid_gains_i !== undefined) {
      resolved.pid_gains_i = msg.pid_gains_i;
    }
    else {
      resolved.pid_gains_i = 0.0
    }

    if (msg.pid_gains_d !== undefined) {
      resolved.pid_gains_d = msg.pid_gains_d;
    }
    else {
      resolved.pid_gains_d = 0.0
    }

    return resolved;
    }
};

// Constants for message
SeActuatorCommand.Constants = {
  MODE_NA: 0,
  MODE_FREEZE: 1,
  MODE_DISABLE: 2,
  MODE_CURRENT: 3,
  MODE_MOTOR_POSITION: 4,
  MODE_MOTOR_VELOCITY: 5,
  MODE_GEAR_POSITION: 6,
  MODE_GEAR_VELOCITY: 7,
  MODE_JOINT_POSITION: 8,
  MODE_JOINT_VELOCITY: 9,
  MODE_JOINT_TORQUE: 10,
  MODE_JOINT_POSITION_VELOCITY: 11,
  MODE_JOINT_POSITION_VELOCITY_TORQUE: 12,
  MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS: 13,
}

module.exports = SeActuatorCommand;
