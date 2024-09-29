;; Auto-generated. Do not edit!


(when (boundp 'multi_agent_system::AgentResponse)
  (if (not (find-package "MULTI_AGENT_SYSTEM"))
    (make-package "MULTI_AGENT_SYSTEM"))
  (shadow 'AgentResponse (find-package "MULTI_AGENT_SYSTEM")))
(unless (find-package "MULTI_AGENT_SYSTEM::AGENTRESPONSE")
  (make-package "MULTI_AGENT_SYSTEM::AGENTRESPONSE"))

(in-package "ROS")
;;//! \htmlinclude AgentResponse.msg.html


(defclass multi_agent_system::AgentResponse
  :super ros::object
  :slots (_response _success ))

(defmethod multi_agent_system::AgentResponse
  (:init
   (&key
    ((:response __response) "")
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _response (string __response))
   (setq _success __success)
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get multi_agent_system::AgentResponse :md5sum-) "61c3bd4caa84e668cf48da398a910dc7")
(setf (get multi_agent_system::AgentResponse :datatype-) "multi_agent_system/AgentResponse")
(setf (get multi_agent_system::AgentResponse :definition-)
      "string response
bool success

")



(provide :multi_agent_system/AgentResponse "61c3bd4caa84e668cf48da398a910dc7")


