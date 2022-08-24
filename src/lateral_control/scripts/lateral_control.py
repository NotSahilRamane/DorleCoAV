#!/usr/bin/env python3
import rospy
import math
import numpy as np
from Stanley_LKA import Stanley_LKA
from stanley_controller import State,Stanley
class laleral_control(object):
    def __init__(self,LKA_status, LCA_status,crosstrack_error, heading_angle_road, car_heading, ego_velocity_longitudinal,lateral_acceleration):
        self.lka_state="driver_control"
        if LKA_status==1:
            
            self.lka_state,self.lka_active_status,self.lka_enabled,self.lka_signal,self.lane_departure_warning,self.driver_control,self.driver_control_status,self.partial_braking,self.right_lane_change_status,self.left_lane_change_status=Stanley(LKA_status,car_heading,crosstrack_error,lateral_acceleration,self.lka_state)
            if self.driver_control_status==0:
                self.steering=Stanley_LKA(crosstrack_error, heading_angle_road, car_heading, ego_velocity_longitudinal)
            else:
                self.steering=0 ##Manual_steering(), take input from the driver
            return self.partial_braking, self.steering
        elif LKA_status==0:
            self.driver_braking=0## Manual_braking(), take input from the driver
            self.lka_state,self.lka_active_status,self.lka_enabled,self.lka_signal,self.lane_departure_warning,self.driver_control,self.driver_control_status,self.partial_braking,self.right_lane_change_status,self.left_lane_change_status=Stanley(LKA_status,car_heading,crosstrack_error,lateral_acceleration,self.lka_state)
            self.braking=np.max(self.driver_braking,self.partial_braking)
            self.steering=0 ##Manual_steering(), take input from the driver
            return self.braking, self.steering
        elif LCA_status==1:
            

        