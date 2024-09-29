// Auto-generated. Do not edit!

// (in-package multi_agent_system.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ValidateRequestRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request = null;
    }
    else {
      if (initObj.hasOwnProperty('request')) {
        this.request = initObj.request
      }
      else {
        this.request = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ValidateRequestRequest
    // Serialize message field [request]
    bufferOffset = _serializer.string(obj.request, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ValidateRequestRequest
    let len;
    let data = new ValidateRequestRequest(null);
    // Deserialize message field [request]
    data.request = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.request);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/ValidateRequestRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b13f31f7a0a36901919f7ec0d9f40d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ValidateRequestRequest(null);
    if (msg.request !== undefined) {
      resolved.request = msg.request;
    }
    else {
      resolved.request = ''
    }

    return resolved;
    }
};

class ValidateRequestResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_standard = null;
      this.validation_details = null;
      this.disassembly_plan = null;
    }
    else {
      if (initObj.hasOwnProperty('is_standard')) {
        this.is_standard = initObj.is_standard
      }
      else {
        this.is_standard = false;
      }
      if (initObj.hasOwnProperty('validation_details')) {
        this.validation_details = initObj.validation_details
      }
      else {
        this.validation_details = '';
      }
      if (initObj.hasOwnProperty('disassembly_plan')) {
        this.disassembly_plan = initObj.disassembly_plan
      }
      else {
        this.disassembly_plan = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ValidateRequestResponse
    // Serialize message field [is_standard]
    bufferOffset = _serializer.bool(obj.is_standard, buffer, bufferOffset);
    // Serialize message field [validation_details]
    bufferOffset = _serializer.string(obj.validation_details, buffer, bufferOffset);
    // Serialize message field [disassembly_plan]
    bufferOffset = _serializer.string(obj.disassembly_plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ValidateRequestResponse
    let len;
    let data = new ValidateRequestResponse(null);
    // Deserialize message field [is_standard]
    data.is_standard = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [validation_details]
    data.validation_details = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [disassembly_plan]
    data.disassembly_plan = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.validation_details);
    length += _getByteLength(object.disassembly_plan);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/ValidateRequestResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c9586279621998d8df57d09408c8ff8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_standard
    string validation_details
    string disassembly_plan
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ValidateRequestResponse(null);
    if (msg.is_standard !== undefined) {
      resolved.is_standard = msg.is_standard;
    }
    else {
      resolved.is_standard = false
    }

    if (msg.validation_details !== undefined) {
      resolved.validation_details = msg.validation_details;
    }
    else {
      resolved.validation_details = ''
    }

    if (msg.disassembly_plan !== undefined) {
      resolved.disassembly_plan = msg.disassembly_plan;
    }
    else {
      resolved.disassembly_plan = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ValidateRequestRequest,
  Response: ValidateRequestResponse,
  md5sum() { return 'd6cfda6094fa0f02393921047736849b'; },
  datatype() { return 'multi_agent_system/ValidateRequest'; }
};
