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

class PlanExecutionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plan = null;
    }
    else {
      if (initObj.hasOwnProperty('plan')) {
        this.plan = initObj.plan
      }
      else {
        this.plan = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanExecutionRequest
    // Serialize message field [plan]
    bufferOffset = _serializer.string(obj.plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanExecutionRequest
    let len;
    let data = new PlanExecutionRequest(null);
    // Deserialize message field [plan]
    data.plan = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.plan);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/PlanExecutionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc50643e78ec83e6cd11adcd6225ad09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string plan
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanExecutionRequest(null);
    if (msg.plan !== undefined) {
      resolved.plan = msg.plan;
    }
    else {
      resolved.plan = ''
    }

    return resolved;
    }
};

class PlanExecutionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.execution_details = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('execution_details')) {
        this.execution_details = initObj.execution_details
      }
      else {
        this.execution_details = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanExecutionResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [execution_details]
    bufferOffset = _serializer.string(obj.execution_details, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanExecutionResponse
    let len;
    let data = new PlanExecutionResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [execution_details]
    data.execution_details = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.execution_details);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'multi_agent_system/PlanExecutionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a45bb69cb171fde9d7e78595dbfb9ca7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string execution_details
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanExecutionResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.execution_details !== undefined) {
      resolved.execution_details = msg.execution_details;
    }
    else {
      resolved.execution_details = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: PlanExecutionRequest,
  Response: PlanExecutionResponse,
  md5sum() { return '47835fb064640c5de7faddad2f012e70'; },
  datatype() { return 'multi_agent_system/PlanExecution'; }
};
