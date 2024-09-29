; Auto-generated. Do not edit!


(cl:in-package multi_agent_system-msg)


;//! \htmlinclude UserCommand.msg.html

(cl:defclass <UserCommand> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass UserCommand (<UserCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UserCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UserCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_agent_system-msg:<UserCommand> is deprecated: use multi_agent_system-msg:UserCommand instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <UserCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_agent_system-msg:command-val is deprecated.  Use multi_agent_system-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UserCommand>) ostream)
  "Serializes a message object of type '<UserCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UserCommand>) istream)
  "Deserializes a message object of type '<UserCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UserCommand>)))
  "Returns string type for a message object of type '<UserCommand>"
  "multi_agent_system/UserCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UserCommand)))
  "Returns string type for a message object of type 'UserCommand"
  "multi_agent_system/UserCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UserCommand>)))
  "Returns md5sum for a message object of type '<UserCommand>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UserCommand)))
  "Returns md5sum for a message object of type 'UserCommand"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UserCommand>)))
  "Returns full string definition for message of type '<UserCommand>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UserCommand)))
  "Returns full string definition for message of type 'UserCommand"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UserCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UserCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'UserCommand
    (cl:cons ':command (command msg))
))
