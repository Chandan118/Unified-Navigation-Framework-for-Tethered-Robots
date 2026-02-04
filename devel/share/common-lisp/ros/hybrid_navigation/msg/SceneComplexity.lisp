; Auto-generated. Do not edit!


(cl:in-package hybrid_navigation-msg)


;//! \htmlinclude SceneComplexity.msg.html

(cl:defclass <SceneComplexity> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (scene_type
    :reader scene_type
    :initarg :scene_type
    :type cl:string
    :initform "")
   (complexity_score
    :reader complexity_score
    :initarg :complexity_score
    :type cl:float
    :initform 0.0)
   (obstacle_density
    :reader obstacle_density
    :initarg :obstacle_density
    :type cl:float
    :initform 0.0)
   (trap_detected
    :reader trap_detected
    :initarg :trap_detected
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SceneComplexity (<SceneComplexity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SceneComplexity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SceneComplexity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hybrid_navigation-msg:<SceneComplexity> is deprecated: use hybrid_navigation-msg:SceneComplexity instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SceneComplexity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:header-val is deprecated.  Use hybrid_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'scene_type-val :lambda-list '(m))
(cl:defmethod scene_type-val ((m <SceneComplexity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:scene_type-val is deprecated.  Use hybrid_navigation-msg:scene_type instead.")
  (scene_type m))

(cl:ensure-generic-function 'complexity_score-val :lambda-list '(m))
(cl:defmethod complexity_score-val ((m <SceneComplexity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:complexity_score-val is deprecated.  Use hybrid_navigation-msg:complexity_score instead.")
  (complexity_score m))

(cl:ensure-generic-function 'obstacle_density-val :lambda-list '(m))
(cl:defmethod obstacle_density-val ((m <SceneComplexity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:obstacle_density-val is deprecated.  Use hybrid_navigation-msg:obstacle_density instead.")
  (obstacle_density m))

(cl:ensure-generic-function 'trap_detected-val :lambda-list '(m))
(cl:defmethod trap_detected-val ((m <SceneComplexity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hybrid_navigation-msg:trap_detected-val is deprecated.  Use hybrid_navigation-msg:trap_detected instead.")
  (trap_detected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SceneComplexity>) ostream)
  "Serializes a message object of type '<SceneComplexity>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'scene_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'scene_type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'complexity_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_density))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trap_detected) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SceneComplexity>) istream)
  "Deserializes a message object of type '<SceneComplexity>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'scene_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'scene_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'complexity_score) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_density) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'trap_detected) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SceneComplexity>)))
  "Returns string type for a message object of type '<SceneComplexity>"
  "hybrid_navigation/SceneComplexity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SceneComplexity)))
  "Returns string type for a message object of type 'SceneComplexity"
  "hybrid_navigation/SceneComplexity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SceneComplexity>)))
  "Returns md5sum for a message object of type '<SceneComplexity>"
  "f871fe5893f973c3015c98df1108df3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SceneComplexity)))
  "Returns md5sum for a message object of type 'SceneComplexity"
  "f871fe5893f973c3015c98df1108df3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SceneComplexity>)))
  "Returns full string definition for message of type '<SceneComplexity>"
  (cl:format cl:nil "# Custom message for scene complexity analysis~%Header header~%string scene_type~%float32 complexity_score~%float32 obstacle_density~%bool trap_detected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SceneComplexity)))
  "Returns full string definition for message of type 'SceneComplexity"
  (cl:format cl:nil "# Custom message for scene complexity analysis~%Header header~%string scene_type~%float32 complexity_score~%float32 obstacle_density~%bool trap_detected~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SceneComplexity>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'scene_type))
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SceneComplexity>))
  "Converts a ROS message object to a list"
  (cl:list 'SceneComplexity
    (cl:cons ':header (header msg))
    (cl:cons ':scene_type (scene_type msg))
    (cl:cons ':complexity_score (complexity_score msg))
    (cl:cons ':obstacle_density (obstacle_density msg))
    (cl:cons ':trap_detected (trap_detected msg))
))
