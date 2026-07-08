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

class LegOdometry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.lin_vel_lf = null;
      this.lin_vel_rf = null;
      this.lin_vel_lh = null;
      this.lin_vel_rh = null;
      this.base_velocity = null;
      this.stance_count = null;
      this.base_height_valid = null;
      this.base_height = null;
      this.stance = null;
      this.base_to_foot_position_world_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('lin_vel_lf')) {
        this.lin_vel_lf = initObj.lin_vel_lf
      }
      else {
        this.lin_vel_lf = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_rf')) {
        this.lin_vel_rf = initObj.lin_vel_rf
      }
      else {
        this.lin_vel_rf = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_lh')) {
        this.lin_vel_lh = initObj.lin_vel_lh
      }
      else {
        this.lin_vel_lh = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('lin_vel_rh')) {
        this.lin_vel_rh = initObj.lin_vel_rh
      }
      else {
        this.lin_vel_rh = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('base_velocity')) {
        this.base_velocity = initObj.base_velocity
      }
      else {
        this.base_velocity = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('stance_count')) {
        this.stance_count = initObj.stance_count
      }
      else {
        this.stance_count = 0;
      }
      if (initObj.hasOwnProperty('base_height_valid')) {
        this.base_height_valid = initObj.base_height_valid
      }
      else {
        this.base_height_valid = false;
      }
      if (initObj.hasOwnProperty('base_height')) {
        this.base_height = initObj.base_height
      }
      else {
        this.base_height = 0.0;
      }
      if (initObj.hasOwnProperty('stance')) {
        this.stance = initObj.stance
      }
      else {
        this.stance = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('base_to_foot_position_world_z')) {
        this.base_to_foot_position_world_z = initObj.base_to_foot_position_world_z
      }
      else {
        this.base_to_foot_position_world_z = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LegOdometry
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [lin_vel_lf] has the right length
    if (obj.lin_vel_lf.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_lf - length must be 3')
    }
    // Serialize message field [lin_vel_lf]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_lf, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_rf] has the right length
    if (obj.lin_vel_rf.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_rf - length must be 3')
    }
    // Serialize message field [lin_vel_rf]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_rf, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_lh] has the right length
    if (obj.lin_vel_lh.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_lh - length must be 3')
    }
    // Serialize message field [lin_vel_lh]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_lh, buffer, bufferOffset, 3);
    // Check that the constant length array field [lin_vel_rh] has the right length
    if (obj.lin_vel_rh.length !== 3) {
      throw new Error('Unable to serialize array field lin_vel_rh - length must be 3')
    }
    // Serialize message field [lin_vel_rh]
    bufferOffset = _arraySerializer.float64(obj.lin_vel_rh, buffer, bufferOffset, 3);
    // Check that the constant length array field [base_velocity] has the right length
    if (obj.base_velocity.length !== 3) {
      throw new Error('Unable to serialize array field base_velocity - length must be 3')
    }
    // Serialize message field [base_velocity]
    bufferOffset = _arraySerializer.float64(obj.base_velocity, buffer, bufferOffset, 3);
    // Serialize message field [stance_count]
    bufferOffset = _serializer.uint8(obj.stance_count, buffer, bufferOffset);
    // Serialize message field [base_height_valid]
    bufferOffset = _serializer.bool(obj.base_height_valid, buffer, bufferOffset);
    // Serialize message field [base_height]
    bufferOffset = _serializer.float64(obj.base_height, buffer, bufferOffset);
    // Check that the constant length array field [stance] has the right length
    if (obj.stance.length !== 4) {
      throw new Error('Unable to serialize array field stance - length must be 4')
    }
    // Serialize message field [stance]
    bufferOffset = _arraySerializer.bool(obj.stance, buffer, bufferOffset, 4);
    // Check that the constant length array field [base_to_foot_position_world_z] has the right length
    if (obj.base_to_foot_position_world_z.length !== 4) {
      throw new Error('Unable to serialize array field base_to_foot_position_world_z - length must be 4')
    }
    // Serialize message field [base_to_foot_position_world_z]
    bufferOffset = _arraySerializer.float64(obj.base_to_foot_position_world_z, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LegOdometry
    let len;
    let data = new LegOdometry(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [lin_vel_lf]
    data.lin_vel_lf = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_rf]
    data.lin_vel_rf = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_lh]
    data.lin_vel_lh = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [lin_vel_rh]
    data.lin_vel_rh = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [base_velocity]
    data.base_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [stance_count]
    data.stance_count = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [base_height_valid]
    data.base_height_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [base_height]
    data.base_height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stance]
    data.stance = _arrayDeserializer.bool(buffer, bufferOffset, 4)
    // Deserialize message field [base_to_foot_position_world_z]
    data.base_to_foot_position_world_z = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 166;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/LegOdometry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '569661be22a02d9165c8c394eeb247a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float64[3] lin_vel_lf
    float64[3] lin_vel_rf
    float64[3] lin_vel_lh
    float64[3] lin_vel_rh
    
    float64[3] base_velocity
    
    uint8 stance_count
    bool base_height_valid
    float64 base_height
    bool[4] stance
    float64[4] base_to_foot_position_world_z
    
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
    const resolved = new LegOdometry(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.lin_vel_lf !== undefined) {
      resolved.lin_vel_lf = msg.lin_vel_lf;
    }
    else {
      resolved.lin_vel_lf = new Array(3).fill(0)
    }

    if (msg.lin_vel_rf !== undefined) {
      resolved.lin_vel_rf = msg.lin_vel_rf;
    }
    else {
      resolved.lin_vel_rf = new Array(3).fill(0)
    }

    if (msg.lin_vel_lh !== undefined) {
      resolved.lin_vel_lh = msg.lin_vel_lh;
    }
    else {
      resolved.lin_vel_lh = new Array(3).fill(0)
    }

    if (msg.lin_vel_rh !== undefined) {
      resolved.lin_vel_rh = msg.lin_vel_rh;
    }
    else {
      resolved.lin_vel_rh = new Array(3).fill(0)
    }

    if (msg.base_velocity !== undefined) {
      resolved.base_velocity = msg.base_velocity;
    }
    else {
      resolved.base_velocity = new Array(3).fill(0)
    }

    if (msg.stance_count !== undefined) {
      resolved.stance_count = msg.stance_count;
    }
    else {
      resolved.stance_count = 0
    }

    if (msg.base_height_valid !== undefined) {
      resolved.base_height_valid = msg.base_height_valid;
    }
    else {
      resolved.base_height_valid = false
    }

    if (msg.base_height !== undefined) {
      resolved.base_height = msg.base_height;
    }
    else {
      resolved.base_height = 0.0
    }

    if (msg.stance !== undefined) {
      resolved.stance = msg.stance;
    }
    else {
      resolved.stance = new Array(4).fill(0)
    }

    if (msg.base_to_foot_position_world_z !== undefined) {
      resolved.base_to_foot_position_world_z = msg.base_to_foot_position_world_z;
    }
    else {
      resolved.base_to_foot_position_world_z = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = LegOdometry;
