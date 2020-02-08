; Auto-generated. Do not edit!


(cl:in-package chuchu_onboard-msg)


;//! \htmlinclude Int16ArrayHeader.msg.html

(cl:defclass <Int16ArrayHeader> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Int16ArrayHeader (<Int16ArrayHeader>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Int16ArrayHeader>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Int16ArrayHeader)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chuchu_onboard-msg:<Int16ArrayHeader> is deprecated: use chuchu_onboard-msg:Int16ArrayHeader instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Int16ArrayHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chuchu_onboard-msg:header-val is deprecated.  Use chuchu_onboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Int16ArrayHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chuchu_onboard-msg:data-val is deprecated.  Use chuchu_onboard-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Int16ArrayHeader>) ostream)
  "Serializes a message object of type '<Int16ArrayHeader>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Int16ArrayHeader>) istream)
  "Deserializes a message object of type '<Int16ArrayHeader>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Int16ArrayHeader>)))
  "Returns string type for a message object of type '<Int16ArrayHeader>"
  "chuchu_onboard/Int16ArrayHeader")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Int16ArrayHeader)))
  "Returns string type for a message object of type 'Int16ArrayHeader"
  "chuchu_onboard/Int16ArrayHeader")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Int16ArrayHeader>)))
  "Returns md5sum for a message object of type '<Int16ArrayHeader>"
  "dc47eb145a11cbe02d6c8cb8248a4099")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Int16ArrayHeader)))
  "Returns md5sum for a message object of type 'Int16ArrayHeader"
  "dc47eb145a11cbe02d6c8cb8248a4099")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Int16ArrayHeader>)))
  "Returns full string definition for message of type '<Int16ArrayHeader>"
  (cl:format cl:nil "Header header~%int16[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Int16ArrayHeader)))
  "Returns full string definition for message of type 'Int16ArrayHeader"
  (cl:format cl:nil "Header header~%int16[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Int16ArrayHeader>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Int16ArrayHeader>))
  "Converts a ROS message object to a list"
  (cl:list 'Int16ArrayHeader
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
