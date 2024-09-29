; Auto-generated. Do not edit!


(cl:in-package multi_agent_system-srv)


;//! \htmlinclude StabilityAnalysis-request.msg.html

(cl:defclass <StabilityAnalysis-request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:string
    :initform ""))
)

(cl:defclass StabilityAnalysis-request (<StabilityAnalysis-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StabilityAnalysis-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StabilityAnalysis-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<StabilityAnalysis-request> is deprecated: use multi_agent_system-srv:StabilityAnalysis-request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <StabilityAnalysis-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:task-val is deprecated.  Use multi_agent_system-srv:task instead.")
  (task m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StabilityAnalysis-request>) ostream)
  "Serializes a message object of type '<StabilityAnalysis-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StabilityAnalysis-request>) istream)
  "Deserializes a message object of type '<StabilityAnalysis-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StabilityAnalysis-request>)))
  "Returns string type for a service object of type '<StabilityAnalysis-request>"
  "multi_agent_system/StabilityAnalysisRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StabilityAnalysis-request)))
  "Returns string type for a service object of type 'StabilityAnalysis-request"
  "multi_agent_system/StabilityAnalysisRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StabilityAnalysis-request>)))
  "Returns md5sum for a message object of type '<StabilityAnalysis-request>"
  "118e7fc2e317fb76cba62a2c92e6b05b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StabilityAnalysis-request)))
  "Returns md5sum for a message object of type 'StabilityAnalysis-request"
  "118e7fc2e317fb76cba62a2c92e6b05b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StabilityAnalysis-request>)))
  "Returns full string definition for message of type '<StabilityAnalysis-request>"
  (cl:format cl:nil "string task~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StabilityAnalysis-request)))
  "Returns full string definition for message of type 'StabilityAnalysis-request"
  (cl:format cl:nil "string task~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StabilityAnalysis-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'task))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StabilityAnalysis-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StabilityAnalysis-request
    (cl:cons ':task (task msg))
))
;//! \htmlinclude StabilityAnalysis-response.msg.html

(cl:defclass <StabilityAnalysis-response> (roslisp-msg-protocol:ros-message)
  ((is_safe
    :reader is_safe
    :initarg :is_safe
    :type cl:boolean
    :initform cl:nil)
   (modifications
    :reader modifications
    :initarg :modifications
    :type cl:string
    :initform "")
   (stability_aware_plan
    :reader stability_aware_plan
    :initarg :stability_aware_plan
    :type cl:string
    :initform ""))
)

(cl:defclass StabilityAnalysis-response (<StabilityAnalysis-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StabilityAnalysis-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StabilityAnalysis-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-srv:<StabilityAnalysis-response> is deprecated: use multi_agent_system-srv:StabilityAnalysis-response instead.")))

(cl:ensure-generic-function 'is_safe-val :lambda-list '(m))
(cl:defmethod is_safe-val ((m <StabilityAnalysis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:is_safe-val is deprecated.  Use multi_agent_system-srv:is_safe instead.")
  (is_safe m))

(cl:ensure-generic-function 'modifications-val :lambda-list '(m))
(cl:defmethod modifications-val ((m <StabilityAnalysis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:modifications-val is deprecated.  Use multi_agent_system-srv:modifications instead.")
  (modifications m))

(cl:ensure-generic-function 'stability_aware_plan-val :lambda-list '(m))
(cl:defmethod stability_aware_plan-val ((m <StabilityAnalysis-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-srv:stability_aware_plan-val is deprecated.  Use multi_agent_system-srv:stability_aware_plan instead.")
  (stability_aware_plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StabilityAnalysis-response>) ostream)
  "Serializes a message object of type '<StabilityAnalysis-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_safe) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'modifications))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'modifications))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'stability_aware_plan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'stability_aware_plan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StabilityAnalysis-response>) istream)
  "Deserializes a message object of type '<StabilityAnalysis-response>"
    (cl:setf (cl:slot-value msg 'is_safe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'modifications) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'modifications) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stability_aware_plan) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'stability_aware_plan) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StabilityAnalysis-response>)))
  "Returns string type for a service object of type '<StabilityAnalysis-response>"
  "multi_agent_system/StabilityAnalysisResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StabilityAnalysis-response)))
  "Returns string type for a service object of type 'StabilityAnalysis-response"
  "multi_agent_system/StabilityAnalysisResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StabilityAnalysis-response>)))
  "Returns md5sum for a message object of type '<StabilityAnalysis-response>"
  "118e7fc2e317fb76cba62a2c92e6b05b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StabilityAnalysis-response)))
  "Returns md5sum for a message object of type 'StabilityAnalysis-response"
  "118e7fc2e317fb76cba62a2c92e6b05b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StabilityAnalysis-response>)))
  "Returns full string definition for message of type '<StabilityAnalysis-response>"
  (cl:format cl:nil "bool is_safe~%string modifications~%string stability_aware_plan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StabilityAnalysis-response)))
  "Returns full string definition for message of type 'StabilityAnalysis-response"
  (cl:format cl:nil "bool is_safe~%string modifications~%string stability_aware_plan~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StabilityAnalysis-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'modifications))
     4 (cl:length (cl:slot-value msg 'stability_aware_plan))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StabilityAnalysis-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StabilityAnalysis-response
    (cl:cons ':is_safe (is_safe msg))
    (cl:cons ':modifications (modifications msg))
    (cl:cons ':stability_aware_plan (stability_aware_plan msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StabilityAnalysis)))
  'StabilityAnalysis-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StabilityAnalysis)))
  'StabilityAnalysis-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StabilityAnalysis)))
  "Returns string type for a service object of type '<StabilityAnalysis>"
  "multi_agent_system/StabilityAnalysis")