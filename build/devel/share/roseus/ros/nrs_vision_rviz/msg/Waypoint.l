;; Auto-generated. Do not edit!


(when (boundp 'nrs_vision_rviz::Waypoint)
  (if (not (find-package "NRS_VISION_RVIZ"))
    (make-package "NRS_VISION_RVIZ"))
  (shadow 'Waypoint (find-package "NRS_VISION_RVIZ")))
(unless (find-package "NRS_VISION_RVIZ::WAYPOINT")
  (make-package "NRS_VISION_RVIZ::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass nrs_vision_rviz::Waypoint
  :super ros::object
  :slots (_point _normal ))

(defmethod nrs_vision_rviz::Waypoint
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:normal __normal) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _point __point)
   (setq _normal __normal)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:normal
   (&rest __normal)
   (if (keywordp (car __normal))
       (send* _normal __normal)
     (progn
       (if __normal (setq _normal (car __normal)))
       _normal)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; geometry_msgs/Vector3 _normal
    (send _normal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; geometry_msgs/Vector3 _normal
       (send _normal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; geometry_msgs/Vector3 _normal
     (send _normal :deserialize buf ptr-) (incf ptr- (send _normal :serialization-length))
   ;;
   self)
  )

(setf (get nrs_vision_rviz::Waypoint :md5sum-) "c4ceba53b835bb664bfc7c4c82b561a8")
(setf (get nrs_vision_rviz::Waypoint :datatype-) "nrs_vision_rviz/Waypoint")
(setf (get nrs_vision_rviz::Waypoint :definition-)
      "geometry_msgs/Point point
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



(provide :nrs_vision_rviz/Waypoint "c4ceba53b835bb664bfc7c4c82b561a8")


