from cmath import inf
import math
from threading import Timer
import numpy
import scipy
import pandas
import matplotlib.pyplot as plt
import time

####################################################
class LKA_Controller:
    def __init__(self,lka_signal,heading_angle,lateral_offset,lateral_acceleration,lka_state):
        #used for defining input variables
        self.lka_signal = lka_signal
        self.heading_angle = heading_angle
        self.lateral_offset = lateral_offset
        self.lateral_acceleration = lateral_acceleration
        self.lka_state = lka_state
        #used for defining constants
        self.lane_width = 3.7
        self.constant_acceleration = 1.5
        self.vehicle_length = 4.7

    def lka_state_machine(self):
        if self.lka_state == "driver_control" and self.lka_signal == 1:
            self.lka_state = "lka_initiating"
            self.lka_enabled = 1
            self.lka_active_status = 0
            self.driver_control_status = 0
            self.partial_braking = 0
            self.left_lane_change_status = 0
            self.right_lane_change_status = 0
            self.driver_control = 1
        elif self.lka_state == "lka_initiating":
            if self.lka_signal == 0:
                self.lka_state = "driver_control"
                self.lka_enabled = 0
                self.lka_active_status = 0
                self.driver_control_status = 1
                self.partial_braking = 0
            elif self.lka_signal != 1:
                if ((self.vehicle_length*math.sin(self.heading_angle))+self.lateral_offset) < 0.5*self.lane_width:
                    self.lka_state = "lka_active"
                    self.lka_enabled = 1
                    self.lka_active_status = 1
                    self.driver_control_status = 0
                    self.partial_braking = 0
                    self.lane_departure_warning = 0
                    self.driver_control = 0
                elif ((self.vehicle_length*math.sin(self.heading_angle))+self.lateral_offset) > 0.5*self.lane_width:
                    if self.lateral_offset < 0:
                        self.left_lane_change_status = 1
                    elif self.lateral_offset > 0:
                        self.right_lane_change_status = 1
                    else :
                        self.right_lane_change_status = 0
                        self.left_lane_change_status = 0
                    self.lka_state = "lane_departure_warning"
                    self.lka_enabled = 1
                    self.lka_active_status = 0
                    self.driver_control_status = 1
                    self.partial_braking = 1
                    self.lane_departure_warning = 1
                    self.driver_control = 1
        elif self.lka_state == "lka_active":
            if self.lka_signal == 0:
                self.lka_state = "driver_control"
                self.lka_enabled = 0
                self.lka_active_status = 0
                self.driver_control_status = 1
                self.partial_braking = 0
            elif abs(self.lateral_acceleration) > self.constant_acceleration:
                if self.lateral_offset < 0:
                    self.left_lane_change_status = 1
                elif self.lateral_offset > 0:
                    self.right_lane_change_status = 1
                else :
                    self.right_lane_change_status = 0
                    self.left_lane_change_status = 0
                self.lka_state = "lane_departure_warning"
                self.lka_enabled = 1
                self.lka_active_status = 0
                self.driver_control_status = 1
                self.partial_braking = 1
                self.lane_departure_warning = 1
                self.driver_control = 1
        elif self.lka_state == "lane_departure_warning":
            time.sleep(1.2)
            self.lka_state = "lka_initiating"
            self.lka_enabled = 1
            self.lka_active_status = 0
            self.driver_control_status = 0
            self.partial_braking = 0
            self.left_lane_change_status = 0
            self.right_lane_change_status = 0
            self.driver_control = 1

        return(self.lka_state,self.lka_active_status,self.lka_enabled,self.lka_signal,self.lane_departure_warning,self.driver_control,self.driver_control_status,self.partial_braking,self.right_lane_change_status,self.left_lane_change_status)
