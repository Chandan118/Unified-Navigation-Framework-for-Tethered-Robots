;; Auto-generated. Do not edit!


(when (boundp 'hybrid_navigation::SceneComplexity)
  (if (not (find-package "HYBRID_NAVIGATION"))
    (make-package "HYBRID_NAVIGATION"))
  (shadow 'SceneComplexity (find-package "HYBRID_NAVIGATION")))
(unless (find-package "HYBRID_NAVIGATION::SCENECOMPLEXITY")
  (make-package "HYBRID_NAVIGATION::SCENECOMPLEXITY"))

(in-package "ROS")
;;//! \htmlinclude SceneComplexity.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hybrid_navigation::SceneComplexity
  :super ros::object
  :slots (_header _scene_type _complexity_score _obstacle_density _trap_detected ))

(defmethod hybrid_navigation::SceneComplexity
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:scene_type __scene_type) "")
    ((:complexity_score __complexity_score) 0.0)
    ((:obstacle_density __obstacle_density) 0.0)
    ((:trap_detected __trap_detected) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _scene_type (string __scene_type))
   (setq _complexity_score (float __complexity_score))
   (setq _obstacle_density (float __obstacle_density))
   (setq _trap_detected __trap_detected)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:scene_type
   (&optional __scene_type)
   (if __scene_type (setq _scene_type __scene_type)) _scene_type)
  (:complexity_score
   (&optional __complexity_score)
   (if __complexity_score (setq _complexity_score __complexity_score)) _complexity_score)
  (:obstacle_density
   (&optional __obstacle_density)
   (if __obstacle_density (setq _obstacle_density __obstacle_density)) _obstacle_density)
  (:trap_detected
   (&optional (__trap_detected :null))
   (if (not (eq __trap_detected :null)) (setq _trap_detected __trap_detected)) _trap_detected)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _scene_type
    4 (length _scene_type)
    ;; float32 _complexity_score
    4
    ;; float32 _obstacle_density
    4
    ;; bool _trap_detected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _scene_type
       (write-long (length _scene_type) s) (princ _scene_type s)
     ;; float32 _complexity_score
       (sys::poke _complexity_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _obstacle_density
       (sys::poke _obstacle_density (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _trap_detected
       (if _trap_detected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _scene_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _scene_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _complexity_score
     (setq _complexity_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _obstacle_density
     (setq _obstacle_density (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _trap_detected
     (setq _trap_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get hybrid_navigation::SceneComplexity :md5sum-) "f871fe5893f973c3015c98df1108df3b")
(setf (get hybrid_navigation::SceneComplexity :datatype-) "hybrid_navigation/SceneComplexity")
(setf (get hybrid_navigation::SceneComplexity :definition-)
      "# Custom message for scene complexity analysis
Header header
string scene_type
float32 complexity_score
float32 obstacle_density
bool trap_detected

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



(provide :hybrid_navigation/SceneComplexity "f871fe5893f973c3015c98df1108df3b")


