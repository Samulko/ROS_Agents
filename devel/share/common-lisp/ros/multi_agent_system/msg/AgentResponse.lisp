; Auto-generated. Do not edit!


(cl:in-package multi_agent_system-msg)


;//! \htmlinclude AgentResponse.msg.html

(cl:defclass <AgentResponse> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform "")
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AgentResponse (<AgentResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-msg:<AgentResponse> is deprecated: use multi_agent_system-msg:AgentResponse instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <AgentResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-msg:response-val is deprecated.  Use multi_agent_system-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <AgentResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-msg:success-val is deprecated.  Use multi_agent_system-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentResponse>) ostream)
  "Serializes a message object of type '<AgentResponse>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentResponse>) istream)
  "Deserializes a message object of type '<AgentResponse>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentResponse>)))
  "Returns string type for a message object of type '<AgentResponse>"
  "multi_agent_system/AgentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentResponse)))
  "Returns string type for a message object of type 'AgentResponse"
  "multi_agent_system/AgentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentResponse>)))
  "Returns md5sum for a message object of type '<AgentResponse>"
  "61c3bd4caa84e668cf48da398a910dc7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentResponse)))
  "Returns md5sum for a message object of type 'AgentResponse"
  "61c3bd4caa84e668cf48da398a910dc7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentResponse>)))
  "Returns full string definition for message of type '<AgentResponse>"
  (cl:format cl:nil "string response~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentResponse)))
  "Returns full string definition for message of type 'AgentResponse"
  (cl:format cl:nil "string response~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentResponse>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentResponse
    (cl:cons ':response (response msg))
    (cl:cons ':success (success msg))
))
