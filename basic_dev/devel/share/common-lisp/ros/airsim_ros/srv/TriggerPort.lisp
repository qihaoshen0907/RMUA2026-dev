; Auto-generated. Do not edit!


(cl:in-package airsim_ros-srv)


;//! \htmlinclude TriggerPort-request.msg.html

(cl:defclass <TriggerPort-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:integer
    :initform 0)
   (enter
    :reader enter
    :initarg :enter
    :type cl:boolean
    :initform cl:nil)
   (uselessbelow
    :reader uselessbelow
    :initarg :uselessbelow
    :type cl:string
    :initform "")
   (age
    :reader age
    :initarg :age
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (weight
    :reader weight
    :initarg :weight
    :type cl:float
    :initform 0.0)
   (uselessabove
    :reader uselessabove
    :initarg :uselessabove
    :type cl:string
    :initform ""))
)

(cl:defclass TriggerPort-request (<TriggerPort-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerPort-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerPort-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name airsim_ros-srv:<TriggerPort-request> is deprecated: use airsim_ros-srv:TriggerPort-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:port-val is deprecated.  Use airsim_ros-srv:port instead.")
  (port m))

(cl:ensure-generic-function 'enter-val :lambda-list '(m))
(cl:defmethod enter-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:enter-val is deprecated.  Use airsim_ros-srv:enter instead.")
  (enter m))

(cl:ensure-generic-function 'uselessbelow-val :lambda-list '(m))
(cl:defmethod uselessbelow-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:uselessbelow-val is deprecated.  Use airsim_ros-srv:uselessbelow instead.")
  (uselessbelow m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:age-val is deprecated.  Use airsim_ros-srv:age instead.")
  (age m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:height-val is deprecated.  Use airsim_ros-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:weight-val is deprecated.  Use airsim_ros-srv:weight instead.")
  (weight m))

(cl:ensure-generic-function 'uselessabove-val :lambda-list '(m))
(cl:defmethod uselessabove-val ((m <TriggerPort-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:uselessabove-val is deprecated.  Use airsim_ros-srv:uselessabove instead.")
  (uselessabove m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerPort-request>) ostream)
  "Serializes a message object of type '<TriggerPort-request>"
  (cl:let* ((signed (cl:slot-value msg 'port)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enter) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uselessbelow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uselessbelow))
  (cl:let* ((signed (cl:slot-value msg 'age)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uselessabove))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uselessabove))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerPort-request>) istream)
  "Deserializes a message object of type '<TriggerPort-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'port) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:setf (cl:slot-value msg 'enter) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uselessbelow) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uselessbelow) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'age) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'weight) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uselessabove) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uselessabove) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerPort-request>)))
  "Returns string type for a service object of type '<TriggerPort-request>"
  "airsim_ros/TriggerPortRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerPort-request)))
  "Returns string type for a service object of type 'TriggerPort-request"
  "airsim_ros/TriggerPortRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerPort-request>)))
  "Returns md5sum for a message object of type '<TriggerPort-request>"
  "4b0800f6a99fc8260513e9a5171b5b0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerPort-request)))
  "Returns md5sum for a message object of type 'TriggerPort-request"
  "4b0800f6a99fc8260513e9a5171b5b0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerPort-request>)))
  "Returns full string definition for message of type '<TriggerPort-request>"
  (cl:format cl:nil "# 该接口仅供测试，正式比赛请勿使用~%int64 port~%bool enter~%string uselessbelow~%int64 age~%float64 height~%float64 weight~%string uselessabove~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerPort-request)))
  "Returns full string definition for message of type 'TriggerPort-request"
  (cl:format cl:nil "# 该接口仅供测试，正式比赛请勿使用~%int64 port~%bool enter~%string uselessbelow~%int64 age~%float64 height~%float64 weight~%string uselessabove~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerPort-request>))
  (cl:+ 0
     8
     1
     4 (cl:length (cl:slot-value msg 'uselessbelow))
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'uselessabove))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerPort-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerPort-request
    (cl:cons ':port (port msg))
    (cl:cons ':enter (enter msg))
    (cl:cons ':uselessbelow (uselessbelow msg))
    (cl:cons ':age (age msg))
    (cl:cons ':height (height msg))
    (cl:cons ':weight (weight msg))
    (cl:cons ':uselessabove (uselessabove msg))
))
;//! \htmlinclude TriggerPort-response.msg.html

(cl:defclass <TriggerPort-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TriggerPort-response (<TriggerPort-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerPort-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerPort-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name airsim_ros-srv:<TriggerPort-response> is deprecated: use airsim_ros-srv:TriggerPort-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TriggerPort-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:success-val is deprecated.  Use airsim_ros-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerPort-response>) ostream)
  "Serializes a message object of type '<TriggerPort-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerPort-response>) istream)
  "Deserializes a message object of type '<TriggerPort-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerPort-response>)))
  "Returns string type for a service object of type '<TriggerPort-response>"
  "airsim_ros/TriggerPortResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerPort-response)))
  "Returns string type for a service object of type 'TriggerPort-response"
  "airsim_ros/TriggerPortResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerPort-response>)))
  "Returns md5sum for a message object of type '<TriggerPort-response>"
  "4b0800f6a99fc8260513e9a5171b5b0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerPort-response)))
  "Returns md5sum for a message object of type 'TriggerPort-response"
  "4b0800f6a99fc8260513e9a5171b5b0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerPort-response>)))
  "Returns full string definition for message of type '<TriggerPort-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerPort-response)))
  "Returns full string definition for message of type 'TriggerPort-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerPort-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerPort-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerPort-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TriggerPort)))
  'TriggerPort-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TriggerPort)))
  'TriggerPort-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerPort)))
  "Returns string type for a service object of type '<TriggerPort>"
  "airsim_ros/TriggerPort")