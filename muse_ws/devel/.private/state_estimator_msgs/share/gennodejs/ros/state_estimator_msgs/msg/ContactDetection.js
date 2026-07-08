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

class ContactDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.stance_lf = null;
      this.stance_rf = null;
      this.stance_lh = null;
      this.stance_rh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('stance_lf')) {
        this.stance_lf = initObj.stance_lf
      }
      else {
        this.stance_lf = false;
      }
      if (initObj.hasOwnProperty('stance_rf')) {
        this.stance_rf = initObj.stance_rf
      }
      else {
        this.stance_rf = false;
      }
      if (initObj.hasOwnProperty('stance_lh')) {
        this.stance_lh = initObj.stance_lh
      }
      else {
        this.stance_lh = false;
      }
      if (initObj.hasOwnProperty('stance_rh')) {
        this.stance_rh = initObj.stance_rh
      }
      else {
        this.stance_rh = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ContactDetection
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [stance_lf]
    bufferOffset = _serializer.bool(obj.stance_lf, buffer, bufferOffset);
    // Serialize message field [stance_rf]
    bufferOffset = _serializer.bool(obj.stance_rf, buffer, bufferOffset);
    // Serialize message field [stance_lh]
    bufferOffset = _serializer.bool(obj.stance_lh, buffer, bufferOffset);
    // Serialize message field [stance_rh]
    bufferOffset = _serializer.bool(obj.stance_rh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ContactDetection
    let len;
    let data = new ContactDetection(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [stance_lf]
    data.stance_lf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_rf]
    data.stance_rf = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_lh]
    data.stance_lh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stance_rh]
    data.stance_rh = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/ContactDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '78b512adaaaa7714d41f42af63ebb35b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool stance_lf
    bool stance_rf
    bool stance_lh
    bool stance_rh
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
    const resolved = new ContactDetection(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.stance_lf !== undefined) {
      resolved.stance_lf = msg.stance_lf;
    }
    else {
      resolved.stance_lf = false
    }

    if (msg.stance_rf !== undefined) {
      resolved.stance_rf = msg.stance_rf;
    }
    else {
      resolved.stance_rf = false
    }

    if (msg.stance_lh !== undefined) {
      resolved.stance_lh = msg.stance_lh;
    }
    else {
      resolved.stance_lh = false
    }

    if (msg.stance_rh !== undefined) {
      resolved.stance_rh = msg.stance_rh;
    }
    else {
      resolved.stance_rh = false
    }

    return resolved;
    }
};

module.exports = ContactDetection;
