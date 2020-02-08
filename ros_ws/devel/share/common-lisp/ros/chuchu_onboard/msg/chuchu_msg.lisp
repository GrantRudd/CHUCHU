; Auto-generated. Do not edit!


(cl:in-package chuchu_onboard-msg)


;//! \htmlinclude chuchu_msg.msg.html

(cl:defclass <chuchu_msg> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass chuchu_msg (<chuchu_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chuchu_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chuchu_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chuchu_onboard-msg:<chuchu_msg> is deprecated: use chuchu_onboard-msg:chuchu_msg instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <chuchu_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chuchu_onboard-msg:timestamp-val is deprecated.  Use chuchu_onboard-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <chuchu_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chuchu_onboard-msg:data-val is deprecated.  Use chuchu_onboard-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chuchu_msg>) ostream)
  "Serializes a message object of type '<chuchu_msg>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chuchu_msg>) istream)
  "Deserializes a message object of type '<chuchu_msg>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chuchu_msg>)))
  "Returns string type for a message object of type '<chuchu_msg>"
  "chuchu_onboard/chuchu_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chuchu_msg)))
  "Returns string type for a message object of type 'chuchu_msg"
  "chuchu_onboard/chuchu_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chuchu_msg>)))
  "Returns md5sum for a message object of type '<chuchu_msg>"
  "c9a1856bdb2c29d32baaf26152452c0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chuchu_msg)))
  "Returns md5sum for a message object of type 'chuchu_msg"
  "c9a1856bdb2c29d32baaf26152452c0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chuchu_msg>)))
  "Returns full string definition for message of type '<chuchu_msg>"
  (cl:format cl:nil "time timestamp~%int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chuchu_msg)))
  "Returns full string definition for message of type 'chuchu_msg"
  (cl:format cl:nil "time timestamp~%int16[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chuchu_msg>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chuchu_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'chuchu_msg
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':data (data msg))
))
