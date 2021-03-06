;; Auto-generated. Do not edit!


(when (boundp 'gt_py::TOF_sensor_data)
  (if (not (find-package "GT_PY"))
    (make-package "GT_PY"))
  (shadow 'TOF_sensor_data (find-package "GT_PY")))
(unless (find-package "GT_PY::TOF_SENSOR_DATA")
  (make-package "GT_PY::TOF_SENSOR_DATA"))

(in-package "ROS")
;;//! \htmlinclude TOF_sensor_data.msg.html


(defclass gt_py::TOF_sensor_data
  :super ros::object
  :slots (_range_reading _lux_reading ))

(defmethod gt_py::TOF_sensor_data
  (:init
   (&key
    ((:range_reading __range_reading) (make-array 3 :initial-element 0 :element-type :integer))
    ((:lux_reading __lux_reading) (make-array 3 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _range_reading __range_reading)
   (setq _lux_reading __lux_reading)
   self)
  (:range_reading
   (&optional __range_reading)
   (if __range_reading (setq _range_reading __range_reading)) _range_reading)
  (:lux_reading
   (&optional __lux_reading)
   (if __lux_reading (setq _lux_reading __lux_reading)) _lux_reading)
  (:serialization-length
   ()
   (+
    ;; int32[3] _range_reading
    (* 4    3)
    ;; float32[3] _lux_reading
    (* 4    3)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[3] _range_reading
     (dotimes (i 3)
       (write-long (elt _range_reading i) s)
       )
     ;; float32[3] _lux_reading
     (dotimes (i 3)
       (sys::poke (elt _lux_reading i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[3] _range_reading
   (dotimes (i (length _range_reading))
     (setf (elt _range_reading i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;; float32[3] _lux_reading
   (dotimes (i (length _lux_reading))
     (setf (elt _lux_reading i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get gt_py::TOF_sensor_data :md5sum-) "3c37e3e2c7851686cb8c295f62489702")
(setf (get gt_py::TOF_sensor_data :datatype-) "gt_py/TOF_sensor_data")
(setf (get gt_py::TOF_sensor_data :definition-)
      "int32[3] range_reading
float32[3] lux_reading

")



(provide :gt_py/TOF_sensor_data "3c37e3e2c7851686cb8c295f62489702")


