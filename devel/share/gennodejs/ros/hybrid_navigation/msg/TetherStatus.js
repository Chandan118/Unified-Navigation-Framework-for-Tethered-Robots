// Auto-generated. Do not edit!

// (in-package hybrid_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TetherStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.length = null;
      this.tension = null;
      this.max_length = null;
      this.snag_detected = null;
      this.base_station_pos = null;
      this.robot_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('tension')) {
        this.tension = initObj.tension
      }
      else {
        this.tension = 0.0;
      }
      if (initObj.hasOwnProperty('max_length')) {
        this.max_length = initObj.max_length
      }
      else {
        this.max_length = 0.0;
      }
      if (initObj.hasOwnProperty('snag_detected')) {
        this.snag_detected = initObj.snag_detected
      }
      else {
        this.snag_detected = false;
      }
      if (initObj.hasOwnProperty('base_station_pos')) {
        this.base_station_pos = initObj.base_station_pos
      }
      else {
        this.base_station_pos = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('robot_pos')) {
        this.robot_pos = initObj.robot_pos
      }
      else {
        this.robot_pos = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TetherStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float32(obj.length, buffer, bufferOffset);
    // Serialize message field [tension]
    bufferOffset = _serializer.float32(obj.tension, buffer, bufferOffset);
    // Serialize message field [max_length]
    bufferOffset = _serializer.float32(obj.max_length, buffer, bufferOffset);
    // Serialize message field [snag_detected]
    bufferOffset = _serializer.bool(obj.snag_detected, buffer, bufferOffset);
    // Serialize message field [base_station_pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.base_station_pos, buffer, bufferOffset);
    // Serialize message field [robot_pos]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.robot_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TetherStatus
    let len;
    let data = new TetherStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tension]
    data.tension = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_length]
    data.max_length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [snag_detected]
    data.snag_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [base_station_pos]
    data.base_station_pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_pos]
    data.robot_pos = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 61;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hybrid_navigation/TetherStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e63c602ea59303866ec3d11befde5db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Custom message for tether status
    Header header
    float32 length
    float32 tension
    float32 max_length
    bool snag_detected
    geometry_msgs/Point base_station_pos
    geometry_msgs/Point robot_pos
    
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
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
    const resolved = new TetherStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.tension !== undefined) {
      resolved.tension = msg.tension;
    }
    else {
      resolved.tension = 0.0
    }

    if (msg.max_length !== undefined) {
      resolved.max_length = msg.max_length;
    }
    else {
      resolved.max_length = 0.0
    }

    if (msg.snag_detected !== undefined) {
      resolved.snag_detected = msg.snag_detected;
    }
    else {
      resolved.snag_detected = false
    }

    if (msg.base_station_pos !== undefined) {
      resolved.base_station_pos = geometry_msgs.msg.Point.Resolve(msg.base_station_pos)
    }
    else {
      resolved.base_station_pos = new geometry_msgs.msg.Point()
    }

    if (msg.robot_pos !== undefined) {
      resolved.robot_pos = geometry_msgs.msg.Point.Resolve(msg.robot_pos)
    }
    else {
      resolved.robot_pos = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = TetherStatus;
