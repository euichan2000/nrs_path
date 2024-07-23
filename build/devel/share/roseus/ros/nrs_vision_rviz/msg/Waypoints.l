;; Auto-generated. Do not edit!


(when (boundp 'nrs_vision_rviz::Waypoints)
  (if (not (find-package "NRS_VISION_RVIZ"))
    (make-package "NRS_VISION_RVIZ"))
  (shadow 'Waypoints (find-package "NRS_VISION_RVIZ")))
(unless (find-package "NRS_VISION_RVIZ::WAYPOINTS")
  (make-package "NRS_VISION_RVIZ::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html


(defclass nrs_vision_rviz::Waypoints
  :super ros::object
  :slots (_waypoints ))

(defmethod nrs_vision_rviz::Waypoints
  (:init
   (&key
    ((:waypoints __waypoints) ())
    )
   (send-super :init)
   (setq _waypoints __waypoints)
   self)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:serialization-length
   ()
   (+
    ;; nrs_vision_rviz/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nrs_vision_rviz/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nrs_vision_rviz/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance nrs_vision_rviz::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get nrs_vision_rviz::Waypoints :md5sum-) "b54c421015efab0dad487797f3eb1230")
(setf (get nrs_vision_rviz::Waypoints :datatype-) "nrs_vision_rviz/Waypoints")
(setf (get nrs_vision_rviz::Waypoints :definition-)
      "Waypoint[] waypoints

================================================================================
MSG: nrs_vision_rviz/Waypoint
geometry_msgs/Point point
geometry_msgs/Vector3 normal

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :nrs_vision_rviz/Waypoints "b54c421015efab0dad487797f3eb1230")


