; Auto-generated. Do not edit!


(cl:in-package airsim_ros-srv)


;//! \htmlinclude DebugSphere-request.msg.html

(cl:defclass <DebugSphere-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DebugSphere-request (<DebugSphere-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DebugSphere-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DebugSphere-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name airsim_ros-srv:<DebugSphere-request> is deprecated: use airsim_ros-srv:DebugSphere-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DebugSphere-request>) ostream)
  "Serializes a message object of type '<DebugSphere-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DebugSphere-request>) istream)
  "Deserializes a message object of type '<DebugSphere-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DebugSphere-request>)))
  "Returns string type for a service object of type '<DebugSphere-request>"
  "airsim_ros/DebugSphereRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugSphere-request)))
  "Returns string type for a service object of type 'DebugSphere-request"
  "airsim_ros/DebugSphereRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DebugSphere-request>)))
  "Returns md5sum for a message object of type '<DebugSphere-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DebugSphere-request)))
  "Returns md5sum for a message object of type 'DebugSphere-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DebugSphere-request>)))
  "Returns full string definition for message of type '<DebugSphere-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DebugSphere-request)))
  "Returns full string definition for message of type 'DebugSphere-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DebugSphere-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DebugSphere-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DebugSphere-request
))
;//! \htmlinclude DebugSphere-response.msg.html

(cl:defclass <DebugSphere-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DebugSphere-response (<DebugSphere-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DebugSphere-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DebugSphere-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name airsim_ros-srv:<DebugSphere-response> is deprecated: use airsim_ros-srv:DebugSphere-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DebugSphere-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader airsim_ros-srv:success-val is deprecated.  Use airsim_ros-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DebugSphere-response>) ostream)
  "Serializes a message object of type '<DebugSphere-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DebugSphere-response>) istream)
  "Deserializes a message object of type '<DebugSphere-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DebugSphere-response>)))
  "Returns string type for a service object of type '<DebugSphere-response>"
  "airsim_ros/DebugSphereResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugSphere-response)))
  "Returns string type for a service object of type 'DebugSphere-response"
  "airsim_ros/DebugSphereResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DebugSphere-response>)))
  "Returns md5sum for a message object of type '<DebugSphere-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DebugSphere-response)))
  "Returns md5sum for a message object of type 'DebugSphere-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DebugSphere-response>)))
  "Returns full string definition for message of type '<DebugSphere-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DebugSphere-response)))
  "Returns full string definition for message of type 'DebugSphere-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DebugSphere-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DebugSphere-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DebugSphere-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DebugSphere)))
  'DebugSphere-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DebugSphere)))
  'DebugSphere-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DebugSphere)))
  "Returns string type for a service object of type '<DebugSphere>"
  "airsim_ros/DebugSphere")