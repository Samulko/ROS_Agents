
(cl:in-package :asdf)

(defsystem "multi_agent_system-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AgentResponse" :depends-on ("_package_AgentResponse"))
    (:file "_package_AgentResponse" :depends-on ("_package"))
    (:file "UserCommand" :depends-on ("_package_UserCommand"))
    (:file "_package_UserCommand" :depends-on ("_package"))
  ))