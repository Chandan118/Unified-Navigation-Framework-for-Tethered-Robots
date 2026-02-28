// Auto-generated. Do not edit!

// (in-package hybrid_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SceneComplexity {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.scene_type = null;
      this.complexity_score = null;
      this.obstacle_density = null;
      this.trap_detected = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('scene_type')) {
        this.scene_type = initObj.scene_type
      }
      else {
        this.scene_type = '';
      }
      if (initObj.hasOwnProperty('complexity_score')) {
        this.complexity_score = initObj.complexity_score
      }
      else {
        this.complexity_score = 0.0;
      }
      if (initObj.hasOwnProperty('obstacle_density')) {
        this.obstacle_density = initObj.obstacle_density
      }
      else {
        this.obstacle_density = 0.0;
      }
      if (initObj.hasOwnProperty('trap_detected')) {
        this.trap_detected = initObj.trap_detected
      }
      else {
        this.trap_detected = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SceneComplexity
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [scene_type]
    bufferOffset = _serializer.string(obj.scene_type, buffer, bufferOffset);
    // Serialize message field [complexity_score]
    bufferOffset = _serializer.float32(obj.complexity_score, buffer, bufferOffset);
    // Serialize message field [obstacle_density]
    bufferOffset = _serializer.float32(obj.obstacle_density, buffer, bufferOffset);
    // Serialize message field [trap_detected]
    bufferOffset = _serializer.bool(obj.trap_detected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SceneComplexity
    let len;
    let data = new SceneComplexity(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [scene_type]
    data.scene_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [complexity_score]
    data.complexity_score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obstacle_density]
    data.obstacle_density = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [trap_detected]
    data.trap_detected = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.scene_type);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hybrid_navigation/SceneComplexity';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f871fe5893f973c3015c98df1108df3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Custom message for scene complexity analysis
    Header header
    string scene_type
    float32 complexity_score
    float32 obstacle_density
    bool trap_detected
    
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
    const resolved = new SceneComplexity(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.scene_type !== undefined) {
      resolved.scene_type = msg.scene_type;
    }
    else {
      resolved.scene_type = ''
    }

    if (msg.complexity_score !== undefined) {
      resolved.complexity_score = msg.complexity_score;
    }
    else {
      resolved.complexity_score = 0.0
    }

    if (msg.obstacle_density !== undefined) {
      resolved.obstacle_density = msg.obstacle_density;
    }
    else {
      resolved.obstacle_density = 0.0
    }

    if (msg.trap_detected !== undefined) {
      resolved.trap_detected = msg.trap_detected;
    }
    else {
      resolved.trap_detected = false
    }

    return resolved;
    }
};

module.exports = SceneComplexity;
