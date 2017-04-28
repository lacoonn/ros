// Auto-generated. Do not edit!

// (in-package knu_ros_lecture.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class knuRosLecture {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.int32Data = null;
      this.float32Data = null;
      this.float64Data = null;
      this.stringData = null;
    }
    else {
      if (initObj.hasOwnProperty('int32Data')) {
        this.int32Data = initObj.int32Data
      }
      else {
        this.int32Data = 0;
      }
      if (initObj.hasOwnProperty('float32Data')) {
        this.float32Data = initObj.float32Data
      }
      else {
        this.float32Data = 0.0;
      }
      if (initObj.hasOwnProperty('float64Data')) {
        this.float64Data = initObj.float64Data
      }
      else {
        this.float64Data = 0.0;
      }
      if (initObj.hasOwnProperty('stringData')) {
        this.stringData = initObj.stringData
      }
      else {
        this.stringData = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type knuRosLecture
    // Serialize message field [int32Data]
    bufferOffset = _serializer.int32(obj.int32Data, buffer, bufferOffset);
    // Serialize message field [float32Data]
    bufferOffset = _serializer.float32(obj.float32Data, buffer, bufferOffset);
    // Serialize message field [float64Data]
    bufferOffset = _serializer.float64(obj.float64Data, buffer, bufferOffset);
    // Serialize message field [stringData]
    bufferOffset = _serializer.string(obj.stringData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type knuRosLecture
    let len;
    let data = new knuRosLecture(null);
    // Deserialize message field [int32Data]
    data.int32Data = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [float32Data]
    data.float32Data = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [float64Data]
    data.float64Data = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [stringData]
    data.stringData = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.stringData.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'knu_ros_lecture/knuRosLecture';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a47b7b41e31bbd789a10dc658a18ec37';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 int32Data
    float32 float32Data
    float64 float64Data
    string stringData
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new knuRosLecture(null);
    if (msg.int32Data !== undefined) {
      resolved.int32Data = msg.int32Data;
    }
    else {
      resolved.int32Data = 0
    }

    if (msg.float32Data !== undefined) {
      resolved.float32Data = msg.float32Data;
    }
    else {
      resolved.float32Data = 0.0
    }

    if (msg.float64Data !== undefined) {
      resolved.float64Data = msg.float64Data;
    }
    else {
      resolved.float64Data = 0.0
    }

    if (msg.stringData !== undefined) {
      resolved.stringData = msg.stringData;
    }
    else {
      resolved.stringData = ''
    }

    return resolved;
    }
};

module.exports = knuRosLecture;
