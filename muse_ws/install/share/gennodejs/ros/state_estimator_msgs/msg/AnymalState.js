// Auto-generated. Do not edit!

// (in-package state_estimator_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ExtendedJointState = require('./ExtendedJointState.js');
let Contact = require('./Contact.js');
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AnymalState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.pose = null;
      this.twist = null;
      this.joints = null;
      this.contacts = null;
      this.frame_transforms = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new geometry_msgs.msg.TwistStamped();
      }
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new ExtendedJointState();
      }
      if (initObj.hasOwnProperty('contacts')) {
        this.contacts = initObj.contacts
      }
      else {
        this.contacts = [];
      }
      if (initObj.hasOwnProperty('frame_transforms')) {
        this.frame_transforms = initObj.frame_transforms
      }
      else {
        this.frame_transforms = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnymalState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.int8(obj.state, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = geometry_msgs.msg.TwistStamped.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [joints]
    bufferOffset = ExtendedJointState.serialize(obj.joints, buffer, bufferOffset);
    // Serialize message field [contacts]
    // Serialize the length for message field [contacts]
    bufferOffset = _serializer.uint32(obj.contacts.length, buffer, bufferOffset);
    obj.contacts.forEach((val) => {
      bufferOffset = Contact.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [frame_transforms]
    // Serialize the length for message field [frame_transforms]
    bufferOffset = _serializer.uint32(obj.frame_transforms.length, buffer, bufferOffset);
    obj.frame_transforms.forEach((val) => {
      bufferOffset = geometry_msgs.msg.TransformStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnymalState
    let len;
    let data = new AnymalState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = geometry_msgs.msg.TwistStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [joints]
    data.joints = ExtendedJointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [contacts]
    // Deserialize array length for message field [contacts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.contacts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.contacts[i] = Contact.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [frame_transforms]
    // Deserialize array length for message field [frame_transforms]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.frame_transforms = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.frame_transforms[i] = geometry_msgs.msg.TransformStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.pose);
    length += geometry_msgs.msg.TwistStamped.getMessageSize(object.twist);
    length += ExtendedJointState.getMessageSize(object.joints);
    object.contacts.forEach((val) => {
      length += Contact.getMessageSize(val);
    });
    object.frame_transforms.forEach((val) => {
      length += geometry_msgs.msg.TransformStamped.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'state_estimator_msgs/AnymalState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bee66925f9d04663ad1cd7f4a496f93';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Anymal state including generalized coordinates, velocities, and contacts.
    std_msgs/Header header
    
    int8 STATE_ERROR_SENSOR=-3
    int8 STATE_ERROR_ESTIMATOR=-2
    int8 STATE_ERROR_UNKNOWN=-1
    int8 STATE_OK=0
    int8 STATE_UNINITIALIZED=1
    int8 state
    
    # Position of the base with respect to the odom frame, and orientation from base to odom.
    geometry_msgs/PoseStamped pose
    
    # Linear velocity and local angular velocity expressed in base frame.
    geometry_msgs/TwistStamped twist
    ExtendedJointState joints
    Contact[] contacts
    geometry_msgs/TransformStamped[] frame_transforms
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/TwistStamped
    # A twist with reference coordinate frame and timestamp
    Header header
    Twist twist
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    MSG: state_estimator_msgs/ExtendedJointState
    # sensor_msgs/JointState with an additional field for acceleration.
    std_msgs/Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] acceleration
    float64[] effort
    
    ================================================================================
    MSG: state_estimator_msgs/Contact
    # Contact definition.
    uint8 STATE_OPEN=0
    uint8 STATE_CLOSED=1
    uint8 STATE_SLIPPING=2
    
    std_msgs/Header header
    string name
    uint8 state
    geometry_msgs/Wrench wrench
    geometry_msgs/Point position
    geometry_msgs/Vector3 normal
    float64 frictionCoefficient
    float64 restitutionCoefficient
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: geometry_msgs/TransformStamped
    # This expresses a transform from coordinate frame header.frame_id
    # to the coordinate frame child_frame_id
    #
    # This message is mostly used by the 
    # <a href="http://wiki.ros.org/tf">tf</a> package. 
    # See its documentation for more information.
    
    Header header
    string child_frame_id # the frame id of the child frame
    Transform transform
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnymalState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.PoseStamped.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.twist !== undefined) {
      resolved.twist = geometry_msgs.msg.TwistStamped.Resolve(msg.twist)
    }
    else {
      resolved.twist = new geometry_msgs.msg.TwistStamped()
    }

    if (msg.joints !== undefined) {
      resolved.joints = ExtendedJointState.Resolve(msg.joints)
    }
    else {
      resolved.joints = new ExtendedJointState()
    }

    if (msg.contacts !== undefined) {
      resolved.contacts = new Array(msg.contacts.length);
      for (let i = 0; i < resolved.contacts.length; ++i) {
        resolved.contacts[i] = Contact.Resolve(msg.contacts[i]);
      }
    }
    else {
      resolved.contacts = []
    }

    if (msg.frame_transforms !== undefined) {
      resolved.frame_transforms = new Array(msg.frame_transforms.length);
      for (let i = 0; i < resolved.frame_transforms.length; ++i) {
        resolved.frame_transforms[i] = geometry_msgs.msg.TransformStamped.Resolve(msg.frame_transforms[i]);
      }
    }
    else {
      resolved.frame_transforms = []
    }

    return resolved;
    }
};

// Constants for message
AnymalState.Constants = {
  STATE_ERROR_SENSOR: -3,
  STATE_ERROR_ESTIMATOR: -2,
  STATE_ERROR_UNKNOWN: -1,
  STATE_OK: 0,
  STATE_UNINITIALIZED: 1,
}

module.exports = AnymalState;
