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

class StabilityAnalysisRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task = null;
    }
    else {
      if (initObj.hasOwnProperty('task')) {
        this.task = initObj.task
      }
      else {
        this.task = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StabilityAnalysisRequest
    // Serialize message field [task]
    bufferOffset = _serializer.string(obj.task, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StabilityAnalysisRequest
    let len;
    let data = new StabilityAnalysisRequest(null);
    // Deserialize message field [task]
    data.task = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.task);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/StabilityAnalysisRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ece8f504419f7ca4d91b277e47ff617';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string task
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StabilityAnalysisRequest(null);
    if (msg.task !== undefined) {
      resolved.task = msg.task;
    }
    else {
      resolved.task = ''
    }

    return resolved;
    }
};

class StabilityAnalysisResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_safe = null;
      this.modifications = null;
      this.stability_aware_plan = null;
    }
    else {
      if (initObj.hasOwnProperty('is_safe')) {
        this.is_safe = initObj.is_safe
      }
      else {
        this.is_safe = false;
      }
      if (initObj.hasOwnProperty('modifications')) {
        this.modifications = initObj.modifications
      }
      else {
        this.modifications = '';
      }
      if (initObj.hasOwnProperty('stability_aware_plan')) {
        this.stability_aware_plan = initObj.stability_aware_plan
      }
      else {
        this.stability_aware_plan = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StabilityAnalysisResponse
    // Serialize message field [is_safe]
    bufferOffset = _serializer.bool(obj.is_safe, buffer, bufferOffset);
    // Serialize message field [modifications]
    bufferOffset = _serializer.string(obj.modifications, buffer, bufferOffset);
    // Serialize message field [stability_aware_plan]
    bufferOffset = _serializer.string(obj.stability_aware_plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StabilityAnalysisResponse
    let len;
    let data = new StabilityAnalysisResponse(null);
    // Deserialize message field [is_safe]
    data.is_safe = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [modifications]
    data.modifications = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [stability_aware_plan]
    data.stability_aware_plan = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.modifications);
    length += _getByteLength(object.stability_aware_plan);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/StabilityAnalysisResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4301bd8d3f19ce7a86b1da814ac6f070';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_safe
    string modifications
    string stability_aware_plan
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StabilityAnalysisResponse(null);
    if (msg.is_safe !== undefined) {
      resolved.is_safe = msg.is_safe;
    }
    else {
      resolved.is_safe = false
    }

    if (msg.modifications !== undefined) {
      resolved.modifications = msg.modifications;
    }
    else {
      resolved.modifications = ''
    }

    if (msg.stability_aware_plan !== undefined) {
      resolved.stability_aware_plan = msg.stability_aware_plan;
    }
    else {
      resolved.stability_aware_plan = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: StabilityAnalysisRequest,
  Response: StabilityAnalysisResponse,
  md5sum() { return '118e7fc2e317fb76cba62a2c92e6b05b'; },
  datatype() { return 'multi_agent_system/StabilityAnalysis'; }
};
