;; Auto-generated. Do not edit!


(when (boundp 'multi_agent_system::UserCommand)
  (if (not (find-package "MULTI_AGENT_SYSTEM"))
    (make-package "MULTI_AGENT_SYSTEM"))
  (shadow 'UserCommand (find-package "MULTI_AGENT_SYSTEM")))
(unless (find-package "MULTI_AGENT_SYSTEM::USERCOMMAND")
  (make-package "MULTI_AGENT_SYSTEM::USERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude UserCommand.msg.html


(defclass multi_agent_system::UserCommand
  :super ros::object
  :slots (_command ))

(defmethod multi_agent_system::UserCommand
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get multi_agent_system::UserCommand :md5sum-) "cba5e21e920a3a2b7b375cb65b64cdea")
(setf (get multi_agent_system::UserCommand :datatype-) "multi_agent_system/UserCommand")
(setf (get multi_agent_system::UserCommand :definition-)
      "string command

")



(provide :multi_agent_system/UserCommand "cba5e21e920a3a2b7b375cb65b64cdea")


