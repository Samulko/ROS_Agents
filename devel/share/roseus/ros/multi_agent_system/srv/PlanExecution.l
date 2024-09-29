;; Auto-generated. Do not edit!


(when (boundp 'multi_agent_system::PlanExecution)
  (if (not (find-package "MULTI_AGENT_SYSTEM"))
    (make-package "MULTI_AGENT_SYSTEM"))
  (shadow 'PlanExecution (find-package "MULTI_AGENT_SYSTEM")))
(unless (find-package "MULTI_AGENT_SYSTEM::PLANEXECUTION")
  (make-package "MULTI_AGENT_SYSTEM::PLANEXECUTION"))
(unless (find-package "MULTI_AGENT_SYSTEM::PLANEXECUTIONREQUEST")
  (make-package "MULTI_AGENT_SYSTEM::PLANEXECUTIONREQUEST"))
(unless (find-package "MULTI_AGENT_SYSTEM::PLANEXECUTIONRESPONSE")
  (make-package "MULTI_AGENT_SYSTEM::PLANEXECUTIONRESPONSE"))

(in-package "ROS")





(defclass multi_agent_system::PlanExecutionRequest
  :super ros::object
  :slots (_plan ))

(defmethod multi_agent_system::PlanExecutionRequest
  (:init
   (&key
    ((:plan __plan) "")
    )
   (send-super :init)
   (setq _plan (string __plan))
   self)
  (:plan
   (&optional __plan)
   (if __plan (setq _plan __plan)) _plan)
  (:serialization-length
   ()
   (+
    ;; string _plan
    4 (length _plan)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _plan
       (write-long (length _plan) s) (princ _plan s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _plan
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plan (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::PlanExecutionResponse
  :super ros::object
  :slots (_success _execution_details ))

(defmethod multi_agent_system::PlanExecutionResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:execution_details __execution_details) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _execution_details (string __execution_details))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:execution_details
   (&optional __execution_details)
   (if __execution_details (setq _execution_details __execution_details)) _execution_details)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _execution_details
    4 (length _execution_details)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _execution_details
       (write-long (length _execution_details) s) (princ _execution_details s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _execution_details
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _execution_details (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::PlanExecution
  :super ros::object
  :slots ())

(setf (get multi_agent_system::PlanExecution :md5sum-) "47835fb064640c5de7faddad2f012e70")
(setf (get multi_agent_system::PlanExecution :datatype-) "multi_agent_system/PlanExecution")
(setf (get multi_agent_system::PlanExecution :request) multi_agent_system::PlanExecutionRequest)
(setf (get multi_agent_system::PlanExecution :response) multi_agent_system::PlanExecutionResponse)

(defmethod multi_agent_system::PlanExecutionRequest
  (:response () (instance multi_agent_system::PlanExecutionResponse :init)))

(setf (get multi_agent_system::PlanExecutionRequest :md5sum-) "47835fb064640c5de7faddad2f012e70")
(setf (get multi_agent_system::PlanExecutionRequest :datatype-) "multi_agent_system/PlanExecutionRequest")
(setf (get multi_agent_system::PlanExecutionRequest :definition-)
      "string plan
---
bool success
string execution_details
")

(setf (get multi_agent_system::PlanExecutionResponse :md5sum-) "47835fb064640c5de7faddad2f012e70")
(setf (get multi_agent_system::PlanExecutionResponse :datatype-) "multi_agent_system/PlanExecutionResponse")
(setf (get multi_agent_system::PlanExecutionResponse :definition-)
      "string plan
---
bool success
string execution_details
")



(provide :multi_agent_system/PlanExecution "47835fb064640c5de7faddad2f012e70")


