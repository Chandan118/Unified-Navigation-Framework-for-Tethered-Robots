;; Auto-generated. Do not edit!


(when (boundp 'hybrid_navigation::TetherStatus)
  (if (not (find-package "HYBRID_NAVIGATION"))
    (make-package "HYBRID_NAVIGATION"))
  (shadow 'TetherStatus (find-package "HYBRID_NAVIGATION")))
(unless (find-package "HYBRID_NAVIGATION::TETHERSTATUS")
  (make-package "HYBRID_NAVIGATION::TETHERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TetherStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hybrid_navigation::TetherStatus
  :super ros::object
  :slots (_header _length _tension _max_length _snag_detected _base_station_pos _robot_pos ))

(defmethod hybrid_navigation::TetherStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:length __length) 0.0)
    ((:tension __tension) 0.0)
    ((:max_length __max_length) 0.0)
    ((:snag_detected __snag_detected) nil)
    ((:base_station_pos __base_station_pos) (instance geometry_msgs::Point :init))
    ((:robot_pos __robot_pos) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _length (float __length))
   (setq _tension (float __tension))
   (setq _max_length (float __max_length))
   (setq _snag_detected __snag_detected)
   (setq _base_station_pos __base_station_pos)
   (setq _robot_pos __robot_pos)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:tension
   (&optional __tension)
   (if __tension (setq _tension __tension)) _tension)
  (:max_length
   (&optional __max_length)
   (if __max_length (setq _max_length __max_length)) _max_length)
  (:snag_detected
   (&optional (__snag_detected :null))
   (if (not (eq __snag_detected :null)) (setq _snag_detected __snag_detected)) _snag_detected)
  (:base_station_pos
   (&rest __base_station_pos)
   (if (keywordp (car __base_station_pos))
       (send* _base_station_pos __base_station_pos)
     (progn
       (if __base_station_pos (setq _base_station_pos (car __base_station_pos)))
       _base_station_pos)))
  (:robot_pos
   (&rest __robot_pos)
   (if (keywordp (car __robot_pos))
       (send* _robot_pos __robot_pos)
     (progn
       (if __robot_pos (setq _robot_pos (car __robot_pos)))
       _robot_pos)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _length
    4
    ;; float32 _tension
    4
    ;; float32 _max_length
    4
    ;; bool _snag_detected
    1
    ;; geometry_msgs/Point _base_station_pos
    (send _base_station_pos :serialization-length)
    ;; geometry_msgs/Point _robot_pos
    (send _robot_pos :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _length
       (sys::poke _length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tension
       (sys::poke _tension (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_length
       (sys::poke _max_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _snag_detected
       (if _snag_detected (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _base_station_pos
       (send _base_station_pos :serialize s)
     ;; geometry_msgs/Point _robot_pos
       (send _robot_pos :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _length
     (setq _length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tension
     (setq _tension (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_length
     (setq _max_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _snag_detected
     (setq _snag_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _base_station_pos
     (send _base_station_pos :deserialize buf ptr-) (incf ptr- (send _base_station_pos :serialization-length))
   ;; geometry_msgs/Point _robot_pos
     (send _robot_pos :deserialize buf ptr-) (incf ptr- (send _robot_pos :serialization-length))
   ;;
   self)
  )

(setf (get hybrid_navigation::TetherStatus :md5sum-) "7e63c602ea59303866ec3d11befde5db")
(setf (get hybrid_navigation::TetherStatus :datatype-) "hybrid_navigation/TetherStatus")
(setf (get hybrid_navigation::TetherStatus :definition-)
      "# Custom message for tether status
Header header
float32 length
float32 tension
float32 max_length
bool snag_detected
geometry_msgs/Point base_station_pos
geometry_msgs/Point robot_pos

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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :hybrid_navigation/TetherStatus "7e63c602ea59303866ec3d11befde5db")


