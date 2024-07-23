
(cl:in-package :asdf)

(defsystem "nrs_vision_rviz-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Waypoint" :depends-on ("_package_Waypoint"))
    (:file "_package_Waypoint" :depends-on ("_package"))
    (:file "Waypoints" :depends-on ("_package_Waypoints"))
    (:file "_package_Waypoints" :depends-on ("_package"))
  ))