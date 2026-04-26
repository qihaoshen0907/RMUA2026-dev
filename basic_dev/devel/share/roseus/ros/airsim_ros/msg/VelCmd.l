;; Auto-generated. Do not edit!


(when (boundp 'airsim_ros::VelCmd)
  (if (not (find-package "AIRSIM_ROS"))
    (make-package "AIRSIM_ROS"))
  (shadow 'VelCmd (find-package "AIRSIM_ROS")))
(unless (find-package "AIRSIM_ROS::VELCMD")
  (make-package "AIRSIM_ROS::VELCMD"))

(in-package "ROS")
;;//! \htmlinclude VelCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass airsim_ros::VelCmd
  :super ros::object
  :slots (_header _vx _vy _vz _yawRate _va _stop ))

(defmethod airsim_ros::VelCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:yawRate __yawRate) 0.0)
    ((:va __va) 0)
    ((:stop __stop) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _yawRate (float __yawRate))
   (setq _va (round __va))
   (setq _stop (round __stop))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:yawRate
   (&optional __yawRate)
   (if __yawRate (setq _yawRate __yawRate)) _yawRate)
  (:va
   (&optional __va)
   (if __va (setq _va __va)) _va)
  (:stop
   (&optional __stop)
   (if __stop (setq _stop __stop)) _stop)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _vx
    8
    ;; float64 _vy
    8
    ;; float64 _vz
    8
    ;; float64 _yawRate
    8
    ;; uint8 _va
    1
    ;; uint8 _stop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yawRate
       (sys::poke _yawRate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _va
       (write-byte _va s)
     ;; uint8 _stop
       (write-byte _stop s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _vx
     (setq _vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vy
     (setq _vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vz
     (setq _vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yawRate
     (setq _yawRate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _va
     (setq _va (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stop
     (setq _stop (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get airsim_ros::VelCmd :md5sum-) "1a888a503939ba521aed68feee8eab6d")
(setf (get airsim_ros::VelCmd :datatype-) "airsim_ros/VelCmd")
(setf (get airsim_ros::VelCmd :definition-)
      "# Official RMUA simulator definition (see IntelligentUAVChampionshipSimulator/.../VelCmdmsg/VelCmd.msg)
std_msgs/Header header
float64 vx
float64 vy
float64 vz
float64 yawRate
uint8 va
uint8 stop

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :airsim_ros/VelCmd "1a888a503939ba521aed68feee8eab6d")


