; Auto-generated. Do not edit!


(cl:in-package nrs_vision_rviz-msg)


;//! \htmlinclude Waypoints.msg.html

(cl:defclass <Waypoints> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector nrs_vision_rviz-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'nrs_vision_rviz-msg:Waypoint :initial-element (cl:make-instance 'nrs_vision_rviz-msg:Waypoint))))
)

(cl:defclass Waypoints (<Waypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Waypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Waypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nrs_vision_rviz-msg:<Waypoints> is deprecated: use nrs_vision_rviz-msg:Waypoints instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <Waypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nrs_vision_rviz-msg:waypoints-val is deprecated.  Use nrs_vision_rviz-msg:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Waypoints>) ostream)
  "Serializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Waypoints>) istream)
  "Deserializes a message object of type '<Waypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nrs_vision_rviz-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Waypoints>)))
  "Returns string type for a message object of type '<Waypoints>"
  "nrs_vision_rviz/Waypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Waypoints)))
  "Returns string type for a message object of type 'Waypoints"
  "nrs_vision_rviz/Waypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Waypoints>)))
  "Returns md5sum for a message object of type '<Waypoints>"
  "b54c421015efab0dad487797f3eb1230")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Waypoints)))
  "Returns md5sum for a message object of type 'Waypoints"
  "b54c421015efab0dad487797f3eb1230")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Waypoints>)))
  "Returns full string definition for message of type '<Waypoints>"
  (cl:format cl:nil "Waypoint[] waypoints~%~%================================================================================~%MSG: nrs_vision_rviz/Waypoint~%geometry_msgs/Point point~%geometry_msgs/Vector3 normal~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Waypoints)))
  "Returns full string definition for message of type 'Waypoints"
  (cl:format cl:nil "Waypoint[] waypoints~%~%================================================================================~%MSG: nrs_vision_rviz/Waypoint~%geometry_msgs/Point point~%geometry_msgs/Vector3 normal~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Waypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Waypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'Waypoints
    (cl:cons ':waypoints (waypoints msg))
))
