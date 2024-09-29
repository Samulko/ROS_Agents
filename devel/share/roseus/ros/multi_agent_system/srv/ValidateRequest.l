;; Auto-generated. Do not edit!


(when (boundp 'multi_agent_system::ValidateRequest)
  (if (not (find-package "MULTI_AGENT_SYSTEM"))
    (make-package "MULTI_AGENT_SYSTEM"))
  (shadow 'ValidateRequest (find-package "MULTI_AGENT_SYSTEM")))
(unless (find-package "MULTI_AGENT_SYSTEM::VALIDATEREQUEST")
  (make-package "MULTI_AGENT_SYSTEM::VALIDATEREQUEST"))
(unless (find-package "MULTI_AGENT_SYSTEM::VALIDATEREQUESTREQUEST")
  (make-package "MULTI_AGENT_SYSTEM::VALIDATEREQUESTREQUEST"))
(unless (find-package "MULTI_AGENT_SYSTEM::VALIDATEREQUESTRESPONSE")
  (make-package "MULTI_AGENT_SYSTEM::VALIDATEREQUESTRESPONSE"))

(in-package "ROS")





(defclass multi_agent_system::ValidateRequestRequest
  :super ros::object
  :slots (_request ))

(defmethod multi_agent_system::ValidateRequestRequest
  (:init
   (&key
    ((:request __request) "")
    )
   (send-super :init)
   (setq _request (string __request))
   self)
  (:request
   (&optional __request)
   (if __request (setq _request __request)) _request)
  (:serialization-length
   ()
   (+
    ;; string _request
    4 (length _request)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _request
       (write-long (length _request) s) (princ _request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _request
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::ValidateRequestResponse
  :super ros::object
  :slots (_is_standard _validation_details _disassembly_plan ))

(defmethod multi_agent_system::ValidateRequestResponse
  (:init
   (&key
    ((:is_standard __is_standard) nil)
    ((:validation_details __validation_details) "")
    ((:disassembly_plan __disassembly_plan) "")
    )
   (send-super :init)
   (setq _is_standard __is_standard)
   (setq _validation_details (string __validation_details))
   (setq _disassembly_plan (string __disassembly_plan))
   self)
  (:is_standard
   (&optional (__is_standard :null))
   (if (not (eq __is_standard :null)) (setq _is_standard __is_standard)) _is_standard)
  (:validation_details
   (&optional __validation_details)
   (if __validation_details (setq _validation_details __validation_details)) _validation_details)
  (:disassembly_plan
   (&optional __disassembly_plan)
   (if __disassembly_plan (setq _disassembly_plan __disassembly_plan)) _disassembly_plan)
  (:serialization-length
   ()
   (+
    ;; bool _is_standard
    1
    ;; string _validation_details
    4 (length _validation_details)
    ;; string _disassembly_plan
    4 (length _disassembly_plan)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_standard
       (if _is_standard (write-byte -1 s) (write-byte 0 s))
     ;; string _validation_details
       (write-long (length _validation_details) s) (princ _validation_details s)
     ;; string _disassembly_plan
       (write-long (length _disassembly_plan) s) (princ _disassembly_plan s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_standard
     (setq _is_standard (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _validation_details
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _validation_details (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _disassembly_plan
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _disassembly_plan (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass multi_agent_system::ValidateRequest
  :super ros::object
  :slots ())

(setf (get multi_agent_system::ValidateRequest :md5sum-) "d6cfda6094fa0f02393921047736849b")
(setf (get multi_agent_system::ValidateRequest :datatype-) "multi_agent_system/ValidateRequest")
(setf (get multi_agent_system::ValidateRequest :request) multi_agent_system::ValidateRequestRequest)
(setf (get multi_agent_system::ValidateRequest :response) multi_agent_system::ValidateRequestResponse)

(defmethod multi_agent_system::ValidateRequestRequest
  (:response () (instance multi_agent_system::ValidateRequestResponse :init)))

(setf (get multi_agent_system::ValidateRequestRequest :md5sum-) "d6cfda6094fa0f02393921047736849b")
(setf (get multi_agent_system::ValidateRequestRequest :datatype-) "multi_agent_system/ValidateRequestRequest")
(setf (get multi_agent_system::ValidateRequestRequest :definition-)
      "string request
---
bool is_standard
string validation_details
string disassembly_plan

")

(setf (get multi_agent_system::ValidateRequestResponse :md5sum-) "d6cfda6094fa0f02393921047736849b")
(setf (get multi_agent_system::ValidateRequestResponse :datatype-) "multi_agent_system/ValidateRequestResponse")
(setf (get multi_agent_system::ValidateRequestResponse :definition-)
      "string request
---
bool is_standard
string validation_details
string disassembly_plan

")



(provide :multi_agent_system/ValidateRequest "d6cfda6094fa0f02393921047736849b")


