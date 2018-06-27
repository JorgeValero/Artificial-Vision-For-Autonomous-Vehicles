// Auto-generated. Do not edit!

// (in-package traffic_sign_recognition.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class sign {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sign_type = null;
      this.area = null;
    }
    else {
      if (initObj.hasOwnProperty('sign_type')) {
        this.sign_type = initObj.sign_type
      }
      else {
        this.sign_type = 0.0;
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sign
    // Serialize message field [sign_type]
    bufferOffset = _serializer.float32(obj.sign_type, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = _serializer.float64(obj.area, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sign
    let len;
    let data = new sign(null);
    // Deserialize message field [sign_type]
    data.sign_type = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traffic_sign_recognition/sign';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '80d74564147e7ccb5048d7289eea8101';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 sign_type
    float64 area
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sign(null);
    if (msg.sign_type !== undefined) {
      resolved.sign_type = msg.sign_type;
    }
    else {
      resolved.sign_type = 0.0
    }

    if (msg.area !== undefined) {
      resolved.area = msg.area;
    }
    else {
      resolved.area = 0.0
    }

    return resolved;
    }
};

module.exports = sign;
