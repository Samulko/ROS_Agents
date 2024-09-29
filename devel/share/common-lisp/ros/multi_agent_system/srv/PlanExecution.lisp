; Auto-generated. Do not edit!


(cl:in-package multi_agent_system-srv)


;//! \htmlinclude PlanExecution-request.msg.html

(cl:defclass <PlanExecution-request> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type cl:string
    :initform ""))
)

(cl:defclass PlanExecution-request (<PlanExecution-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanExecution-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanExecution-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<PlanExecution-request> is deprecated: use multi_agent_system-srv:PlanExecution-request instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <PlanExecution-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:plan-val is deprecated.  Use multi_agent_system-srv:plan instead.")
  (plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanExecution-request>) ostream)
  "Serializes a message object of type '<PlanExecution-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanExecution-request>) istream)
  "Deserializes a message object of type '<PlanExecution-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'plan) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanExecution-request>)))
  "Returns string type for a service object of type '<PlanExecution-request>"
  "multi_agent_system/PlanExecutionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanExecution-request)))
  "Returns string type for a service object of type 'PlanExecution-request"
  "multi_agent_system/PlanExecutionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanExecution-request>)))
  "Returns md5sum for a message object of type '<PlanExecution-request>"
  "47835fb064640c5de7faddad2f012e70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanExecution-request)))
  "Returns md5sum for a message object of type 'PlanExecution-request"
  "47835fb064640c5de7faddad2f012e70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanExecution-request>)))
  "Returns full string definition for message of type '<PlanExecution-request>"
  (cl:format cl:nil "string plan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanExecution-request)))
  "Returns full string definition for message of type 'PlanExecution-request"
  (cl:format cl:nil "string plan~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanExecution-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'plan))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanExecution-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanExecution-request
    (cl:cons ':plan (plan msg))
))
;//! \htmlinclude PlanExecution-response.msg.html

(cl:defclass <PlanExecution-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (execution_details
    :reader execution_details
    :initarg :execution_details
    :type cl:string
    :initform ""))
)

(cl:defclass PlanExecution-response (<PlanExecution-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanExecution-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanExecution-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<PlanExecution-response> is deprecated: use multi_agent_system-srv:PlanExecution-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlanExecution-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:success-val is deprecated.  Use multi_agent_system-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'execution_details-val :lambda-list '(m))
(cl:defmethod execution_details-val ((m <PlanExecution-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:execution_details-val is deprecated.  Use multi_agent_system-srv:execution_details instead.")
  (execution_details m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanExecution-response>) ostream)
  "Serializes a message object of type '<PlanExecution-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'execution_details))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'execution_details))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanExecution-response>) istream)
  "Deserializes a message object of type '<PlanExecution-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'execution_details) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'execution_details) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanExecution-response>)))
  "Returns string type for a service object of type '<PlanExecution-response>"
  "multi_agent_system/PlanExecutionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanExecution-response)))
  "Returns string type for a service object of type 'PlanExecution-response"
  "multi_agent_system/PlanExecutionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanExecution-response>)))
  "Returns md5sum for a message object of type '<PlanExecution-response>"
  "47835fb064640c5de7faddad2f012e70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanExecution-response)))
  "Returns md5sum for a message object of type 'PlanExecution-response"
  "47835fb064640c5de7faddad2f012e70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanExecution-response>)))
  "Returns full string definition for message of type '<PlanExecution-response>"
  (cl:format cl:nil "bool success~%string execution_details~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanExecution-response)))
  "Returns full string definition for message of type 'PlanExecution-response"
  (cl:format cl:nil "bool success~%string execution_details~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanExecution-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'execution_details))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanExecution-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanExecution-response
    (cl:cons ':success (success msg))
    (cl:cons ':execution_details (execution_details msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanExecution)))
  'PlanExecution-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanExecution)))
  'PlanExecution-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanExecution)))
  "Returns string type for a service object of type '<PlanExecution>"
  "multi_agent_system/PlanExecution")