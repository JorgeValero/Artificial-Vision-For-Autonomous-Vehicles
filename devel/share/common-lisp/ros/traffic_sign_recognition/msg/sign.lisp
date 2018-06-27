; Auto-generated. Do not edit!


(cl:in-package traffic_sign_recognition-msg)


;//! \htmlinclude sign.msg.html

(cl:defclass <sign> (roslisp-msg-protocol:ros-message)
  ((sign_type
    :reader sign_type
    :initarg :sign_type
    :type cl:float
    :initform 0.0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0))
)

(cl:defclass sign (<sign>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sign>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sign)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traffic_sign_recognition-msg:<sign> is deprecated: use traffic_sign_recognition-msg:sign instead.")))

(cl:ensure-generic-function 'sign_type-val :lambda-list '(m))
(cl:defmethod sign_type-val ((m <sign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traffic_sign_recognition-msg:sign_type-val is deprecated.  Use traffic_sign_recognition-msg:sign_type instead.")
  (sign_type m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <sign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traffic_sign_recognition-msg:area-val is deprecated.  Use traffic_sign_recognition-msg:area instead.")
  (area m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sign>) ostream)
  "Serializes a message object of type '<sign>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sign_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sign>) istream)
  "Deserializes a message object of type '<sign>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sign_type) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sign>)))
  "Returns string type for a message object of type '<sign>"
  "traffic_sign_recognition/sign")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sign)))
  "Returns string type for a message object of type 'sign"
  "traffic_sign_recognition/sign")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sign>)))
  "Returns md5sum for a message object of type '<sign>"
  "80d74564147e7ccb5048d7289eea8101")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sign)))
  "Returns md5sum for a message object of type 'sign"
  "80d74564147e7ccb5048d7289eea8101")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sign>)))
  "Returns full string definition for message of type '<sign>"
  (cl:format cl:nil "float32 sign_type~%float64 area~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sign)))
  "Returns full string definition for message of type 'sign"
  (cl:format cl:nil "float32 sign_type~%float64 area~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sign>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sign>))
  "Converts a ROS message object to a list"
  (cl:list 'sign
    (cl:cons ':sign_type (sign_type msg))
    (cl:cons ':area (area msg))
))
