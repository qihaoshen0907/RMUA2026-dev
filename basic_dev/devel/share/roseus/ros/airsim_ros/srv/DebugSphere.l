;; Auto-generated. Do not edit!


(when (boundp 'airsim_ros::DebugSphere)
  (if (not (find-package "AIRSIM_ROS"))
    (make-package "AIRSIM_ROS"))
  (shadow 'DebugSphere (find-package "AIRSIM_ROS")))
(unless (find-package "AIRSIM_ROS::DEBUGSPHERE")
  (make-package "AIRSIM_ROS::DEBUGSPHERE"))
(unless (find-package "AIRSIM_ROS::DEBUGSPHEREREQUEST")
  (make-package "AIRSIM_ROS::DEBUGSPHEREREQUEST"))
(unless (find-package "AIRSIM_ROS::DEBUGSPHERERESPONSE")
  (make-package "AIRSIM_ROS::DEBUGSPHERERESPONSE"))

(in-package "ROS")





(defclass airsim_ros::DebugSphereRequest
  :super ros::object
  :slots ())

(defmethod airsim_ros::DebugSphereRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass airsim_ros::DebugSphereResponse
  :super ros::object
  :slots (_success ))

(defmethod airsim_ros::DebugSphereResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass airsim_ros::DebugSphere
  :super ros::object
  :slots ())

(setf (get airsim_ros::DebugSphere :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get airsim_ros::DebugSphere :datatype-) "airsim_ros/DebugSphere")
(setf (get airsim_ros::DebugSphere :request) airsim_ros::DebugSphereRequest)
(setf (get airsim_ros::DebugSphere :response) airsim_ros::DebugSphereResponse)

(defmethod airsim_ros::DebugSphereRequest
  (:response () (instance airsim_ros::DebugSphereResponse :init)))

(setf (get airsim_ros::DebugSphereRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get airsim_ros::DebugSphereRequest :datatype-) "airsim_ros/DebugSphereRequest")
(setf (get airsim_ros::DebugSphereRequest :definition-)
      "---
bool success
")

(setf (get airsim_ros::DebugSphereResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get airsim_ros::DebugSphereResponse :datatype-) "airsim_ros/DebugSphereResponse")
(setf (get airsim_ros::DebugSphereResponse :definition-)
      "---
bool success
")



(provide :airsim_ros/DebugSphere "358e233cde0c8a8bcfea4ce193f8fc15")


