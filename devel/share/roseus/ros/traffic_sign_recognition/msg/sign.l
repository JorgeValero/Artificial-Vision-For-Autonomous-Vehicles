;; Auto-generated. Do not edit!


(when (boundp 'traffic_sign_recognition::sign)
  (if (not (find-package "TRAFFIC_SIGN_RECOGNITION"))
    (make-package "TRAFFIC_SIGN_RECOGNITION"))
  (shadow 'sign (find-package "TRAFFIC_SIGN_RECOGNITION")))
(unless (find-package "TRAFFIC_SIGN_RECOGNITION::SIGN")
  (make-package "TRAFFIC_SIGN_RECOGNITION::SIGN"))

(in-package "ROS")
;;//! \htmlinclude sign.msg.html


(defclass traffic_sign_recognition::sign
  :super ros::object
  :slots (_sign_type _area ))

(defmethod traffic_sign_recognition::sign
  (:init
   (&key
    ((:sign_type __sign_type) 0.0)
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _sign_type (float __sign_type))
   (setq _area (float __area))
   self)
  (:sign_type
   (&optional __sign_type)
   (if __sign_type (setq _sign_type __sign_type)) _sign_type)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; float32 _sign_type
    4
    ;; float64 _area
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _sign_type
       (sys::poke _sign_type (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _area
       (sys::poke _area (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _sign_type
     (setq _sign_type (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _area
     (setq _area (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get traffic_sign_recognition::sign :md5sum-) "80d74564147e7ccb5048d7289eea8101")
(setf (get traffic_sign_recognition::sign :datatype-) "traffic_sign_recognition/sign")
(setf (get traffic_sign_recognition::sign :definition-)
      "float32 sign_type
float64 area

")



(provide :traffic_sign_recognition/sign "80d74564147e7ccb5048d7289eea8101")


