;; Auto-generated. Do not edit!


(when (boundp 'imu_pkg::InsState15)
  (if (not (find-package "IMU_PKG"))
    (make-package "IMU_PKG"))
  (shadow 'InsState15 (find-package "IMU_PKG")))
(unless (find-package "IMU_PKG::INSSTATE15")
  (make-package "IMU_PKG::INSSTATE15"))

(in-package "ROS")
;;//! \htmlinclude InsState15.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass imu_pkg::InsState15
  :super ros::object
  :slots (_header _x _y _z _roll _pitch _yaw _vx _vy _vz _vroll _vpitch _vyaw _ax _ay _az _valid_mask ))

(defmethod imu_pkg::InsState15
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:vroll __vroll) 0.0)
    ((:vpitch __vpitch) 0.0)
    ((:vyaw __vyaw) 0.0)
    ((:ax __ax) 0.0)
    ((:ay __ay) 0.0)
    ((:az __az) 0.0)
    ((:valid_mask __valid_mask) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _vroll (float __vroll))
   (setq _vpitch (float __vpitch))
   (setq _vyaw (float __vyaw))
   (setq _ax (float __ax))
   (setq _ay (float __ay))
   (setq _az (float __az))
   (setq _valid_mask (round __valid_mask))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:vroll
   (&optional __vroll)
   (if __vroll (setq _vroll __vroll)) _vroll)
  (:vpitch
   (&optional __vpitch)
   (if __vpitch (setq _vpitch __vpitch)) _vpitch)
  (:vyaw
   (&optional __vyaw)
   (if __vyaw (setq _vyaw __vyaw)) _vyaw)
  (:ax
   (&optional __ax)
   (if __ax (setq _ax __ax)) _ax)
  (:ay
   (&optional __ay)
   (if __ay (setq _ay __ay)) _ay)
  (:az
   (&optional __az)
   (if __az (setq _az __az)) _az)
  (:valid_mask
   (&optional __valid_mask)
   (if __valid_mask (setq _valid_mask __valid_mask)) _valid_mask)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; float64 _vx
    8
    ;; float64 _vy
    8
    ;; float64 _vz
    8
    ;; float64 _vroll
    8
    ;; float64 _vpitch
    8
    ;; float64 _vyaw
    8
    ;; float64 _ax
    8
    ;; float64 _ay
    8
    ;; float64 _az
    8
    ;; uint32 _valid_mask
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vroll
       (sys::poke _vroll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vpitch
       (sys::poke _vpitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vyaw
       (sys::poke _vyaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ax
       (sys::poke _ax (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ay
       (sys::poke _ay (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _az
       (sys::poke _az (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _valid_mask
       (write-long _valid_mask s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vx
     (setq _vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vy
     (setq _vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vz
     (setq _vz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vroll
     (setq _vroll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vpitch
     (setq _vpitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vyaw
     (setq _vyaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ax
     (setq _ax (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ay
     (setq _ay (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _az
     (setq _az (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _valid_mask
     (setq _valid_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get imu_pkg::InsState15 :md5sum-) "80480d447f042cf02f557bec1c2627b8")
(setf (get imu_pkg::InsState15 :datatype-) "imu_pkg/InsState15")
(setf (get imu_pkg::InsState15 :definition-)
      "# Header ROS chuẩn để đóng dấu thời gian và frame cho bản tin 15 state.
std_msgs/Header header

# 15 state bên dưới giữ đúng thứ tự state vector của robot_localization.
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 vx
float64 vy
float64 vz
float64 vroll
float64 vpitch
float64 vyaw
float64 ax
float64 ay
float64 az

# Mỗi bit trong valid_mask cho biết state tương ứng có được giải mã hay không.
# Bit 0 -> x, bit 1 -> y, ..., bit 14 -> az.
uint32 valid_mask

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



(provide :imu_pkg/InsState15 "80480d447f042cf02f557bec1c2627b8")


