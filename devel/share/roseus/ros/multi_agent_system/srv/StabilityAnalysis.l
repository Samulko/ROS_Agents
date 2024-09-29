;; Auto-generated. Do not edit!


(when (boundp 'multi_agent_system::StabilityAnalysis)
  (if (not (find-package "MULTI_AGENT_SYSTEM"))
    (make-package "MULTI_AGENT_SYSTEM"))
  (shadow 'StabilityAnalysis (find-package "MULTI_AGENT_SYSTEM")))
(unless (find-package "MULTI_AGENT_SYSTEM::STABILITYANALYSIS")
  (make-package "MULTI_AGENT_SYSTEM::STABILITYANALYSIS"))
(unless (find-package "MULTI_AGENT_SYSTEM::STABILITYANALYSISREQUEST")
  (make-package "MULTI_AGENT_SYSTEM::STABILITYANALYSISREQUEST"))
(unless (find-package "MULTI_AGENT_SYSTEM::STABILITYANALYSISRESPONSE")
  (make-package "MULTI_AGENT_SYSTEM::STABILITYANALYSISRESPONSE"))

(in-package "ROS")





(defclass multi_agent_system::StabilityAnalysisRequest
  :super ros::object
  :slots (_task ))

(defmethod multi_agent_system::StabilityAnalysisRequest
  (:init
   (&key
    ((:task __task) "")
    )
   (send-super :init)
   (setq _task (string __task))
   self)
  (:task
   (&optional __task)
   (if __task (setq _task __task)) _task)
  (:serialization-length
   ()
   (+
    ;; string _task
    4 (length _task)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _task
       (write-long (length _task) s) (princ _task s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _task
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::StabilityAnalysisResponse
  :super ros::object
  :slots (_is_safe _modifications _stability_aware_plan ))

(defmethod multi_agent_system::StabilityAnalysisResponse
  (:init
   (&key
    ((:is_safe __is_safe) nil)
    ((:modifications __modifications) "")
    ((:stability_aware_plan __stability_aware_plan) "")
    )
   (send-super :init)
   (setq _is_safe __is_safe)
   (setq _modifications (string __modifications))
   (setq _stability_aware_plan (string __stability_aware_plan))
   self)
  (:is_safe
   (&optional (__is_safe :null))
   (if (not (eq __is_safe :null)) (setq _is_safe __is_safe)) _is_safe)
  (:modifications
   (&optional __modifications)
   (if __modifications (setq _modifications __modifications)) _modifications)
  (:stability_aware_plan
   (&optional __stability_aware_plan)
   (if __stability_aware_plan (setq _stability_aware_plan __stability_aware_plan)) _stability_aware_plan)
  (:serialization-length
   ()
   (+
    ;; bool _is_safe
    1
    ;; string _modifications
    4 (length _modifications)
    ;; string _stability_aware_plan
    4 (length _stability_aware_plan)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_safe
       (if _is_safe (write-byte -1 s) (write-byte 0 s))
     ;; string _modifications
       (write-long (length _modifications) s) (princ _modifications s)
     ;; string _stability_aware_plan
       (write-long (length _stability_aware_plan) s) (princ _stability_aware_plan s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_safe
     (setq _is_safe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _modifications
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _modifications (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _stability_aware_plan
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _stability_aware_plan (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::StabilityAnalysis
  :super ros::object
  :slots ())

(setf (get multi_agent_system::StabilityAnalysis :md5sum-) "118e7fc2e317fb76cba62a2c92e6b05b")
(setf (get multi_agent_system::StabilityAnalysis :datatype-) "multi_agent_system/StabilityAnalysis")
(setf (get multi_agent_system::StabilityAnalysis :request) multi_agent_system::StabilityAnalysisRequest)
(setf (get multi_agent_system::StabilityAnalysis :response) multi_agent_system::StabilityAnalysisResponse)

(defmethod multi_agent_system::StabilityAnalysisRequest
  (:response () (instance multi_agent_system::StabilityAnalysisResponse :init)))

(setf (get multi_agent_system::StabilityAnalysisRequest :md5sum-) "118e7fc2e317fb76cba62a2c92e6b05b")
(setf (get multi_agent_system::StabilityAnalysisRequest :datatype-) "multi_agent_system/StabilityAnalysisRequest")
(setf (get multi_agent_system::StabilityAnalysisRequest :definition-)
      "string task
---
bool is_safe
string modifications
string stability_aware_plan

")

(setf (get multi_agent_system::StabilityAnalysisResponse :md5sum-) "118e7fc2e317fb76cba62a2c92e6b05b")
(setf (get multi_agent_system::StabilityAnalysisResponse :datatype-) "multi_agent_system/StabilityAnalysisResponse")
(setf (get multi_agent_system::StabilityAnalysisResponse :definition-)
      "string task
---
bool is_safe
string modifications
string stability_aware_plan

")



(provide :multi_agent_system/StabilityAnalysis "118e7fc2e317fb76cba62a2c92e6b05b")


