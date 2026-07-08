// Auto-generated. Do not edit!

// (in-package series_elastic_actuator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SeActuatorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.statusword = null;
      this.current = null;
      this.gear_position = null;
      this.gear_velocity = null;
      this.joint_position = null;
      this.joint_velocity = null;
      this.joint_acceleration = null;
      this.joint_torque = null;
      this.imu = null;
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
      if (initObj.hasOwnProperty('statusword')) {
        this.statusword = initObj.statusword
      }
      else {
        this.statusword = 0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('gear_position')) {
        this.gear_position = initObj.gear_position
      }
      else {
        this.gear_position = 0.0;
      }
      if (initObj.hasOwnProperty('gear_velocity')) {
        this.gear_velocity = initObj.gear_velocity
      }
      else {
        this.gear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = 0.0;
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('joint_acceleration')) {
        this.joint_acceleration = initObj.joint_acceleration
      }
      else {
        this.joint_acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('joint_torque')) {
        this.joint_torque = initObj.joint_torque
      }
      else {
        this.joint_torque = 0.0;
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new sensor_msgs.msg.Imu();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SeActuatorState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [statusword]
    bufferOffset = _serializer.uint32(obj.statusword, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Serialize message field [gear_position]
    bufferOffset = _serializer.float64(obj.gear_position, buffer, bufferOffset);
    // Serialize message field [gear_velocity]
    bufferOffset = _serializer.float64(obj.gear_velocity, buffer, bufferOffset);
    // Serialize message field [joint_position]
    bufferOffset = _serializer.float64(obj.joint_position, buffer, bufferOffset);
    // Serialize message field [joint_velocity]
    bufferOffset = _serializer.float64(obj.joint_velocity, buffer, bufferOffset);
    // Serialize message field [joint_acceleration]
    bufferOffset = _serializer.float64(obj.joint_acceleration, buffer, bufferOffset);
    // Serialize message field [joint_torque]
    bufferOffset = _serializer.float64(obj.joint_torque, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SeActuatorState
    let len;
    let data = new SeActuatorState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [statusword]
    data.statusword = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gear_position]
    data.gear_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gear_velocity]
    data.gear_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_position]
    data.joint_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_acceleration]
    data.joint_acceleration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_torque]
    data.joint_torque = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.name);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'series_elastic_actuator_msgs/SeActuatorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5beb885e629c734ee244b26cb1ffcc17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Series-elastic actuator state
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SeActuatorState(null);
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

    if (msg.statusword !== undefined) {
      resolved.statusword = msg.statusword;
    }
    else {
      resolved.statusword = 0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.gear_position !== undefined) {
      resolved.gear_position = msg.gear_position;
    }
    else {
      resolved.gear_position = 0.0
    }

    if (msg.gear_velocity !== undefined) {
      resolved.gear_velocity = msg.gear_velocity;
    }
    else {
      resolved.gear_velocity = 0.0
    }

    if (msg.joint_position !== undefined) {
      resolved.joint_position = msg.joint_position;
    }
    else {
      resolved.joint_position = 0.0
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = 0.0
    }

    if (msg.joint_acceleration !== undefined) {
      resolved.joint_acceleration = msg.joint_acceleration;
    }
    else {
      resolved.joint_acceleration = 0.0
    }

    if (msg.joint_torque !== undefined) {
      resolved.joint_torque = msg.joint_torque;
    }
    else {
      resolved.joint_torque = 0.0
    }

    if (msg.imu !== undefined) {
      resolved.imu = sensor_msgs.msg.Imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new sensor_msgs.msg.Imu()
    }

    return resolved;
    }
};

module.exports = SeActuatorState;
