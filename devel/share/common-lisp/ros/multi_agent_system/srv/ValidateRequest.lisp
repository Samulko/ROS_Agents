; Auto-generated. Do not edit!


(cl:in-package multi_agent_system-srv)


;//! \htmlinclude ValidateRequest-request.msg.html

(cl:defclass <ValidateRequest-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:string
    :initform ""))
)

(cl:defclass ValidateRequest-request (<ValidateRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidateRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidateRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<ValidateRequest-request> is deprecated: use multi_agent_system-srv:ValidateRequest-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <ValidateRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:request-val is deprecated.  Use multi_agent_system-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidateRequest-request>) ostream)
  "Serializes a message object of type '<ValidateRequest-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidateRequest-request>) istream)
  "Deserializes a message object of type '<ValidateRequest-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidateRequest-request>)))
  "Returns string type for a service object of type '<ValidateRequest-request>"
  "multi_agent_system/ValidateRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateRequest-request)))
  "Returns string type for a service object of type 'ValidateRequest-request"
  "multi_agent_system/ValidateRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidateRequest-request>)))
  "Returns md5sum for a message object of type '<ValidateRequest-request>"
  "d6cfda6094fa0f02393921047736849b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidateRequest-request)))
  "Returns md5sum for a message object of type 'ValidateRequest-request"
  "d6cfda6094fa0f02393921047736849b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidateRequest-request>)))
  "Returns full string definition for message of type '<ValidateRequest-request>"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidateRequest-request)))
  "Returns full string definition for message of type 'ValidateRequest-request"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidateRequest-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidateRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidateRequest-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude ValidateRequest-response.msg.html

(cl:defclass <ValidateRequest-response> (roslisp-msg-protocol:ros-message)
  ((is_standard
    :reader is_standard
    :initarg :is_standard
    :type cl:boolean
    :initform cl:nil)
   (validation_details
    :reader validation_details
    :initarg :validation_details
    :type cl:string
    :initform "")
   (disassembly_plan
    :reader disassembly_plan
    :initarg :disassembly_plan
    :type cl:string
    :initform ""))
)

(cl:defclass ValidateRequest-response (<ValidateRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ValidateRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ValidateRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<ValidateRequest-response> is deprecated: use multi_agent_system-srv:ValidateRequest-response instead.")))

(cl:ensure-generic-function 'is_standard-val :lambda-list '(m))
(cl:defmethod is_standard-val ((m <ValidateRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:is_standard-val is deprecated.  Use multi_agent_system-srv:is_standard instead.")
  (is_standard m))

(cl:ensure-generic-function 'validation_details-val :lambda-list '(m))
(cl:defmethod validation_details-val ((m <ValidateRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:validation_details-val is deprecated.  Use multi_agent_system-srv:validation_details instead.")
  (validation_details m))

(cl:ensure-generic-function 'disassembly_plan-val :lambda-list '(m))
(cl:defmethod disassembly_plan-val ((m <ValidateRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:disassembly_plan-val is deprecated.  Use multi_agent_system-srv:disassembly_plan instead.")
  (disassembly_plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ValidateRequest-response>) ostream)
  "Serializes a message object of type '<ValidateRequest-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_standard) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'validation_details))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'validation_details))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'disassembly_plan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'disassembly_plan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ValidateRequest-response>) istream)
  "Deserializes a message object of type '<ValidateRequest-response>"
    (cl:setf (cl:slot-value msg 'is_standard) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'validation_details) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'validation_details) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'disassembly_plan) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'disassembly_plan) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ValidateRequest-response>)))
  "Returns string type for a service object of type '<ValidateRequest-response>"
  "multi_agent_system/ValidateRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateRequest-response)))
  "Returns string type for a service object of type 'ValidateRequest-response"
  "multi_agent_system/ValidateRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ValidateRequest-response>)))
  "Returns md5sum for a message object of type '<ValidateRequest-response>"
  "d6cfda6094fa0f02393921047736849b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ValidateRequest-response)))
  "Returns md5sum for a message object of type 'ValidateRequest-response"
  "d6cfda6094fa0f02393921047736849b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ValidateRequest-response>)))
  "Returns full string definition for message of type '<ValidateRequest-response>"
  (cl:format cl:nil "bool is_standard~%string validation_details~%string disassembly_plan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ValidateRequest-response)))
  "Returns full string definition for message of type 'ValidateRequest-response"
  (cl:format cl:nil "bool is_standard~%string validation_details~%string disassembly_plan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ValidateRequest-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'validation_details))
     4 (cl:length (cl:slot-value msg 'disassembly_plan))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ValidateRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ValidateRequest-response
    (cl:cons ':is_standard (is_standard msg))
    (cl:cons ':validation_details (validation_details msg))
    (cl:cons ':disassembly_plan (disassembly_plan msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ValidateRequest)))
  'ValidateRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ValidateRequest)))
  'ValidateRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ValidateRequest)))
  "Returns string type for a service object of type '<ValidateRequest>"
  "multi_agent_system/ValidateRequest")