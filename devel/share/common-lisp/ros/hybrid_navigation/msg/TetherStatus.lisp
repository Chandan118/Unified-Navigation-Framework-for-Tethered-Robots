; Auto-generated. Do not edit!


(cl:in-package hybrid_navigation-msg)


;//! \htmlinclude TetherStatus.msg.html

(cl:defclass <TetherStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (tension
    :reader tension
    :initarg :tension
    :type cl:float
    :initform 0.0)
   (max_length
    :reader max_length
    :initarg :max_length
    :type cl:float
    :initform 0.0)
   (snag_detected
    :reader snag_detected
    :initarg :snag_detected
    :type cl:boolean
    :initform cl:nil)
   (base_station_pos
    :reader base_station_pos
    :initarg :base_station_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (robot_pos
    :reader robot_pos
    :initarg :robot_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass TetherStatus (<TetherStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TetherStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TetherStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hybrid_navigation-msg:<TetherStatus> is deprecated: use hybrid_navigation-msg:TetherStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:header-val is deprecated.  Use hybrid_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:length-val is deprecated.  Use hybrid_navigation-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'tension-val :lambda-list '(m))
(cl:defmethod tension-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:tension-val is deprecated.  Use hybrid_navigation-msg:tension instead.")
  (tension m))

(cl:ensure-generic-function 'max_length-val :lambda-list '(m))
(cl:defmethod max_length-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:max_length-val is deprecated.  Use hybrid_navigation-msg:max_length instead.")
  (max_length m))

(cl:ensure-generic-function 'snag_detected-val :lambda-list '(m))
(cl:defmethod snag_detected-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:snag_detected-val is deprecated.  Use hybrid_navigation-msg:snag_detected instead.")
  (snag_detected m))

(cl:ensure-generic-function 'base_station_pos-val :lambda-list '(m))
(cl:defmethod base_station_pos-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:base_station_pos-val is deprecated.  Use hybrid_navigation-msg:base_station_pos instead.")
  (base_station_pos m))

(cl:ensure-generic-function 'robot_pos-val :lambda-list '(m))
(cl:defmethod robot_pos-val ((m <TetherStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:robot_pos-val is deprecated.  Use hybrid_navigation-msg:robot_pos instead.")
  (robot_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TetherStatus>) ostream)
  "Serializes a message object of type '<TetherStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tension))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'snag_detected) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base_station_pos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TetherStatus>) istream)
  "Deserializes a message object of type '<TetherStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tension) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_length) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'snag_detected) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base_station_pos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TetherStatus>)))
  "Returns string type for a message object of type '<TetherStatus>"
  "hybrid_navigation/TetherStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TetherStatus)))
  "Returns string type for a message object of type 'TetherStatus"
  "hybrid_navigation/TetherStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TetherStatus>)))
  "Returns md5sum for a message object of type '<TetherStatus>"
  "7e63c602ea59303866ec3d11befde5db")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TetherStatus)))
  "Returns md5sum for a message object of type 'TetherStatus"
  "7e63c602ea59303866ec3d11befde5db")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TetherStatus>)))
  "Returns full string definition for message of type '<TetherStatus>"
  (cl:format cl:nil "# Custom message for tether status~%Header header~%float32 length~%float32 tension~%float32 max_length~%bool snag_detected~%geometry_msgs/Point base_station_pos~%geometry_msgs/Point robot_pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TetherStatus)))
  "Returns full string definition for message of type 'TetherStatus"
  (cl:format cl:nil "# Custom message for tether status~%Header header~%float32 length~%float32 tension~%float32 max_length~%bool snag_detected~%geometry_msgs/Point base_station_pos~%geometry_msgs/Point robot_pos~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TetherStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base_station_pos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TetherStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TetherStatus
    (cl:cons ':header (header msg))
    (cl:cons ':length (length msg))
    (cl:cons ':tension (tension msg))
    (cl:cons ':max_length (max_length msg))
    (cl:cons ':snag_detected (snag_detected msg))
    (cl:cons ':base_station_pos (base_station_pos msg))
    (cl:cons ':robot_pos (robot_pos msg))
))
