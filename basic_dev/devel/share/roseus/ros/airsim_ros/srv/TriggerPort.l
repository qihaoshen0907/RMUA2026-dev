;; Auto-generated. Do not edit!


(when (boundp 'airsim_ros::TriggerPort)
  (if (not (find-package "AIRSIM_ROS"))
    (make-package "AIRSIM_ROS"))
  (shadow 'TriggerPort (find-package "AIRSIM_ROS")))
(unless (find-package "AIRSIM_ROS::TRIGGERPORT")
  (make-package "AIRSIM_ROS::TRIGGERPORT"))
(unless (find-package "AIRSIM_ROS::TRIGGERPORTREQUEST")
  (make-package "AIRSIM_ROS::TRIGGERPORTREQUEST"))
(unless (find-package "AIRSIM_ROS::TRIGGERPORTRESPONSE")
  (make-package "AIRSIM_ROS::TRIGGERPORTRESPONSE"))

(in-package "ROS")





(defclass airsim_ros::TriggerPortRequest
  :super ros::object
  :slots (_port _enter _uselessbelow _age _height _weight _uselessabove ))

(defmethod airsim_ros::TriggerPortRequest
  (:init
   (&key
    ((:port __port) 0)
    ((:enter __enter) nil)
    ((:uselessbelow __uselessbelow) "")
    ((:age __age) 0)
    ((:height __height) 0.0)
    ((:weight __weight) 0.0)
    ((:uselessabove __uselessabove) "")
    )
   (send-super :init)
   (setq _port (round __port))
   (setq _enter __enter)
   (setq _uselessbelow (string __uselessbelow))
   (setq _age (round __age))
   (setq _height (float __height))
   (setq _weight (float __weight))
   (setq _uselessabove (string __uselessabove))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:enter
   (&optional (__enter :null))
   (if (not (eq __enter :null)) (setq _enter __enter)) _enter)
  (:uselessbelow
   (&optional __uselessbelow)
   (if __uselessbelow (setq _uselessbelow __uselessbelow)) _uselessbelow)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:weight
   (&optional __weight)
   (if __weight (setq _weight __weight)) _weight)
  (:uselessabove
   (&optional __uselessabove)
   (if __uselessabove (setq _uselessabove __uselessabove)) _uselessabove)
  (:serialization-length
   ()
   (+
    ;; int64 _port
    8
    ;; bool _enter
    1
    ;; string _uselessbelow
    4 (length _uselessbelow)
    ;; int64 _age
    8
    ;; float64 _height
    8
    ;; float64 _weight
    8
    ;; string _uselessabove
    4 (length _uselessabove)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _port
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _port (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _port) (= (length (_port . bv)) 2)) ;; bignum
              (write-long (ash (elt (_port . bv) 0) 0) s)
              (write-long (ash (elt (_port . bv) 1) -1) s))
             ((and (class _port) (= (length (_port . bv)) 1)) ;; big1
              (write-long (elt (_port . bv) 0) s)
              (write-long (if (>= _port 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _port s)(write-long (if (>= _port 0) 0 #xffffffff) s)))
     ;; bool _enter
       (if _enter (write-byte -1 s) (write-byte 0 s))
     ;; string _uselessbelow
       (write-long (length _uselessbelow) s) (princ _uselessbelow s)
     ;; int64 _age
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _age (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _age) (= (length (_age . bv)) 2)) ;; bignum
              (write-long (ash (elt (_age . bv) 0) 0) s)
              (write-long (ash (elt (_age . bv) 1) -1) s))
             ((and (class _age) (= (length (_age . bv)) 1)) ;; big1
              (write-long (elt (_age . bv) 0) s)
              (write-long (if (>= _age 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _age s)(write-long (if (>= _age 0) 0 #xffffffff) s)))
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _weight
       (sys::poke _weight (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _uselessabove
       (write-long (length _uselessabove) s) (princ _uselessabove s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _port
#+(or :alpha :irix6 :x86_64)
      (setf _port (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _port (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; bool _enter
     (setq _enter (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _uselessbelow
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uselessbelow (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _age
#+(or :alpha :irix6 :x86_64)
      (setf _age (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _age (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _weight
     (setq _weight (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _uselessabove
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uselessabove (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass airsim_ros::TriggerPortResponse
  :super ros::object
  :slots (_success ))

(defmethod airsim_ros::TriggerPortResponse
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

(defclass airsim_ros::TriggerPort
  :super ros::object
  :slots ())

(setf (get airsim_ros::TriggerPort :md5sum-) "4b0800f6a99fc8260513e9a5171b5b0f")
(setf (get airsim_ros::TriggerPort :datatype-) "airsim_ros/TriggerPort")
(setf (get airsim_ros::TriggerPort :request) airsim_ros::TriggerPortRequest)
(setf (get airsim_ros::TriggerPort :response) airsim_ros::TriggerPortResponse)

(defmethod airsim_ros::TriggerPortRequest
  (:response () (instance airsim_ros::TriggerPortResponse :init)))

(setf (get airsim_ros::TriggerPortRequest :md5sum-) "4b0800f6a99fc8260513e9a5171b5b0f")
(setf (get airsim_ros::TriggerPortRequest :datatype-) "airsim_ros/TriggerPortRequest")
(setf (get airsim_ros::TriggerPortRequest :definition-)
      "# 该接口仅供测试，正式比赛请勿使用
int64 port
bool enter
string uselessbelow
int64 age
float64 height
float64 weight
string uselessabove
---
bool success
")

(setf (get airsim_ros::TriggerPortResponse :md5sum-) "4b0800f6a99fc8260513e9a5171b5b0f")
(setf (get airsim_ros::TriggerPortResponse :datatype-) "airsim_ros/TriggerPortResponse")
(setf (get airsim_ros::TriggerPortResponse :definition-)
      "# 该接口仅供测试，正式比赛请勿使用
int64 port
bool enter
string uselessbelow
int64 age
float64 height
float64 weight
string uselessabove
---
bool success
")



(provide :airsim_ros/TriggerPort "4b0800f6a99fc8260513e9a5171b5b0f")


