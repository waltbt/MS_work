import rospkg
import numpy as np

PI = np.pi


from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input
from gt_py.msg import IR_sensor_data
from gt_py.msg import TOF_sensor_data
from gt_py.msg import color_sensor_data

"""
command msg
------------------------
float64[] destination
float64 v
float64 a
bool io_0


position msg
------------------------
float64[] position
bool isReady


gripper_input msg
------------------------
int32 DIGIN 
float64 AIN0
float64 AIN1


IR_sensor_data msg
------------------------
int32[3] sensor_reading

TOF_sensor_data msg
------------------------
int32[3] range_reading
float32[3] lux_reading

"""
