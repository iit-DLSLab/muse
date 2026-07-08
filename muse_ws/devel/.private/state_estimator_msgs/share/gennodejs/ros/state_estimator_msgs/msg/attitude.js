// Auto-generated. Do not edit!

// (in-package state_estimator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class attitude {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.quaternion = null;
      this.roll_deg = null;
      this.pitch_deg = null;
      this.yaw_deg = null;
      this.angular_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('quaternion')) {
        this.quaternion = initObj.quaternion
      }
      else {
        this.quaternion = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('roll_deg')) {
        this.roll_deg = initObj.roll_deg
      }
      else {
        this.roll_deg = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_deg')) {
        this.pitch_deg = initObj.pitch_deg
      }
      else {
        this.pitch_deg = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_deg')) {
        this.yaw_deg = initObj.yaw_deg
      }
      else {
        this.yaw_deg = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type attitude
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [quaternion] has the right length
    if (obj.quaternion.length !== 4) {
      throw new Error('Unable to serialize array field quaternion - length must be 4')
    }
    // Serialize message field [quaternion]
    bufferOffset = _arraySerializer.float64(obj.quaternion, buffer, bufferOffset, 4);
    // Serialize message field [roll_deg]
    bufferOffset = _serializer.float64(obj.roll_deg, buffer, bufferOffset);
    // Serialize message field [pitch_deg]
    bufferOffset = _serializer.float64(obj.pitch_deg, buffer, bufferOffset);
    // Serialize message field [yaw_deg]
    bufferOffset = _serializer.float64(obj.yaw_deg, buffer, bufferOffset);
    // Check that the constant length array field [angular_velocity] has the right length
    if (obj.angular_velocity.length !== 3) {
      throw new Error('Unable to serialize array field angular_velocity - length must be 3')
    }
    // Serialize message field [angular_velocity]
    bufferOffset = _arraySerializer.float64(obj.angular_velocity, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type attitude
    let len;
    let data = new attitude(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [quaternion]
    data.quaternion = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [roll_deg]
    data.roll_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch_deg]
    data.pitch_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_deg]
    data.yaw_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/attitude';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ecef9a2429ca3b3adda042fe513a909a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64[4] quaternion
    float64 roll_deg
    float64 pitch_deg
    float64 yaw_deg 
    float64[3] angular_velocity
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
    const resolved = new attitude(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.quaternion !== undefined) {
      resolved.quaternion = msg.quaternion;
    }
    else {
      resolved.quaternion = new Array(4).fill(0)
    }

    if (msg.roll_deg !== undefined) {
      resolved.roll_deg = msg.roll_deg;
    }
    else {
      resolved.roll_deg = 0.0
    }

    if (msg.pitch_deg !== undefined) {
      resolved.pitch_deg = msg.pitch_deg;
    }
    else {
      resolved.pitch_deg = 0.0
    }

    if (msg.yaw_deg !== undefined) {
      resolved.yaw_deg = msg.yaw_deg;
    }
    else {
      resolved.yaw_deg = 0.0
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = msg.angular_velocity;
    }
    else {
      resolved.angular_velocity = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = attitude;
