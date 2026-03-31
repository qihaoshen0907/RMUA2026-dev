// Auto-generated. Do not edit!

// (in-package airsim_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class TriggerPortRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port = null;
      this.enter = null;
      this.uselessbelow = null;
      this.age = null;
      this.height = null;
      this.weight = null;
      this.uselessabove = null;
    }
    else {
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('enter')) {
        this.enter = initObj.enter
      }
      else {
        this.enter = false;
      }
      if (initObj.hasOwnProperty('uselessbelow')) {
        this.uselessbelow = initObj.uselessbelow
      }
      else {
        this.uselessbelow = '';
      }
      if (initObj.hasOwnProperty('age')) {
        this.age = initObj.age
      }
      else {
        this.age = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = 0.0;
      }
      if (initObj.hasOwnProperty('uselessabove')) {
        this.uselessabove = initObj.uselessabove
      }
      else {
        this.uselessabove = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TriggerPortRequest
    // Serialize message field [port]
    bufferOffset = _serializer.int64(obj.port, buffer, bufferOffset);
    // Serialize message field [enter]
    bufferOffset = _serializer.bool(obj.enter, buffer, bufferOffset);
    // Serialize message field [uselessbelow]
    bufferOffset = _serializer.string(obj.uselessbelow, buffer, bufferOffset);
    // Serialize message field [age]
    bufferOffset = _serializer.int64(obj.age, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [weight]
    bufferOffset = _serializer.float64(obj.weight, buffer, bufferOffset);
    // Serialize message field [uselessabove]
    bufferOffset = _serializer.string(obj.uselessabove, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TriggerPortRequest
    let len;
    let data = new TriggerPortRequest(null);
    // Deserialize message field [port]
    data.port = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [enter]
    data.enter = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [uselessbelow]
    data.uselessbelow = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [age]
    data.age = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [weight]
    data.weight = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [uselessabove]
    data.uselessabove = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.uselessbelow);
    length += _getByteLength(object.uselessabove);
    return length + 41;
  }

  static datatype() {
    // Returns string type for a service object
    return 'airsim_ros/TriggerPortRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '563dfac93c3dedab70779f538911a5e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 该接口仅供测试，正式比赛请勿使用
    int64 port
    bool enter
    string uselessbelow
    int64 age
    float64 height
    float64 weight
    string uselessabove
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TriggerPortRequest(null);
    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.enter !== undefined) {
      resolved.enter = msg.enter;
    }
    else {
      resolved.enter = false
    }

    if (msg.uselessbelow !== undefined) {
      resolved.uselessbelow = msg.uselessbelow;
    }
    else {
      resolved.uselessbelow = ''
    }

    if (msg.age !== undefined) {
      resolved.age = msg.age;
    }
    else {
      resolved.age = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.weight !== undefined) {
      resolved.weight = msg.weight;
    }
    else {
      resolved.weight = 0.0
    }

    if (msg.uselessabove !== undefined) {
      resolved.uselessabove = msg.uselessabove;
    }
    else {
      resolved.uselessabove = ''
    }

    return resolved;
    }
};

class TriggerPortResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TriggerPortResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TriggerPortResponse
    let len;
    let data = new TriggerPortResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'airsim_ros/TriggerPortResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TriggerPortResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: TriggerPortRequest,
  Response: TriggerPortResponse,
  md5sum() { return '4b0800f6a99fc8260513e9a5171b5b0f'; },
  datatype() { return 'airsim_ros/TriggerPort'; }
};
