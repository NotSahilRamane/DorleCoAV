from cmath import inf
import math
from threading import Timer
import numpy
import scipy
import pandas
import matplotlib.pyplot as plt



######################################################################################
class AEB_Controller:
    def __init__(self):
        # used for definition
#         self.relative_dist = relative_dist
#         self.ego_vel = ego_vel
#         self.ACC_set_speed = ACC_set_speed
#         self.ttc = ttc
#         self.ego_acc = ego_acc
        # parameters which we won't need outside the aeb block --- constants
        self.AEB_PB1 = 2
        self.AEB_PB2 = 3
        self.AEB_FB = 3.9
        self.Kp = 1
        self.Kd = 0
        self.Ki = 1
        self.min_def_dist = 1.5
        self.AEB_Headway_Offset = 3
        self.FCW_Reaction_Time = 1.2
        self.FCW_Driver_Deacc = 4
        self.acceleration = 0
        # parameters  which keep on updated every iteration
        self.previous_error = 0                                  # error value for the previous time step 
        self.previous_error_y = 0                                # error in distance for the previous time step
        self.I_previous = 0                                      # I value for the previous time step  
        self.I_previous_y = 0                                    # I_y value for the previous time step
        self.I = 0                                               # I value for the current time step 
        self.I_y = 0                                             # initial value of distance error
        self.FCW_Stopping_Time = inf
        self.PB1_Stopping_Time = inf
        self.PB2_Stopping_Time = inf
        self.FB_Stopping_Time = inf
        self.stop_bool = False
        self.deacc = 0
        self.aeb_status = False
        self.fcw_status = False
        self.velocity_offset = 0
        self.velocity_offset_controlled = 0
        self.ACC_state = 1
        self.ACC_enable = 1
        self.AEB_state = "Default"

        # inputs and parameters before simulation starts 
        # set simulation end time
        dt = 0.1                    # get from ROS   
        initial_rel_dist = min(self.MIO_position, self.distance)
        initial_ego_vel = self.ego_velocity_x
        initial_acc_set_speed = 20
        
        
        

    # check for ttc > 0        we don't need ttc for acc or cc we need it only for aeb 
    def TTC_non_positive(self):
        if (self.ttc <= 0) or (self.ego_vel <= 0):
            self.stop_bool = True
        else:
            self.stop_bool = False
        return self.stop_bool

    def PID(self, error, dt):  
        control_var = 0                                
        #dt = 0.1                    # get from ROS   
        #self.time_step += ent_time           
            #print("i = {}. type = {}".format(i, type(i)))
            #print(error[i])
        P = (self.Kp)*error
        self.I = self.I_previous + (self.Ki)*error*dt           
        D = (self.Kd)*(error-self.previous_error)/dt   
        control_var = P + self.I + D
        return control_var

    def PID_distance(self, error_y, dt):
        control_var_y = 0                                
        #dt = 0.1                    # get from ROS   
        #self.time_step += ent_time           
            #print("i = {}. type = {}".format(i, type(i)))
            #print(error[i])
        P_y = (self.Kp)*error_y
        self.I = self.I_previous_y + (self.Ki)*error_y*dt           
        D_y = (self.Kd)*(error_y-self.previous_error_y)/dt   
        control_var_y = P_y + self.I_y + D_y
        return control_var_y
        
    
    #def calculate_error(self, control, setpoint):
    #    self.error = setpoint - control

    def acc_controller(self):
        
        if self.ACC_enable > 0:
            ACC_acc = 0
            self.velocity_offset = self.ACC_set_speed - self.ego_vel
            error = self.velocity_offset
            sd = (2*self.ego_vel)+self.min_def_dist                             
            y = self.relative_dist - sd
            error_y = y

            if self.ACC_state == 1 and y <= 0:                                                                  
                self.ACC_state = 2
            elif self.ACC_state == 2 and (y >= 0.2*sd and j > 100):                       
                self.ACC_state = 1
            elif self.ACC_state == 2 and (y <= 0 and j > 100):
                self.ACC_state = 3
            elif self.ACC_state == 3 and y >= 0.2*sd:
                self.ACC_state = 2

            if self.ACC_state == 1:
                ACC_acc = self.PID(error, dt)
                self.previous_error = error
            elif self.ACC_state == 2:
                for j in range(1, 101):
                    j += 1
                    self.previous_error = self.ACC_set_speed - self.ego_vel
                    self.previous_error_y = self.relative_dist - (2*self.ego_vel)+self.min_def_dist
                    self.ACC_state += 0.05
            elif self.ACC_state == 3:
                ACC_acc = self.PID_distance(error_y, dt)
                self.previous_error_y = error_y

            if ACC_acc > 1:
                ACC_acc = 1
            elif ACC_acc < -0:
                ACC_acc = 0
            self.acceleration = ACC_acc
        else:
            self.velocity_offset_controlled = self.PID(error, dt)
            self.previous_error = error
            self.acceleration = self.velocity_offset_controlled

        return self.acceleration

    def stopping_time(self):
        reaction_time1 = (self.ego_vel/self.FCW_Driver_Deacc)*1.63
        self.FCW_Stopping_Time = self.FCW_Reaction_Time + reaction_time1
        return self.FCW_Stopping_Time

    def stopping_time_calc(self):
        self.PB1_Stopping_Time = self.ego_vel/self.AEB_PB1
        self.PB2_Stopping_Time = self.ego_vel/self.AEB_PB2
        self.FB_Stopping_Time = self.ego_vel/self.AEB_FB
        return self.PB1_Stopping_Time, self.PB2_Stopping_Time, self.FB_Stopping_Time

    def AEB_state_machine(self):
        if (self.ttc > 0) : 
            if self.AEB_state == "Default" and (self.ttc < self.FCW_Stopping_Time):
                self.AEB_state = "FCW"
                self.aeb_status = 0
                self.fcw_status = 1
                self.deacc = 0
            elif self.AEB_state == "FCW" and (self.ttc > 1.2*self.FCW_Stopping_Time):
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "FCW" and (self.ttc <= self.PB1_Stopping_Time):
                self.AEB_state = "PB1"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_PB1
            elif self.AEB_state == "PB1" and (self.ttc < self.PB2_Stopping_Time):
                self.AEB_state = "PB2"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_PB2
            elif self.AEB_state == "PB1" and self.stop_bool is True:
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "PB2" and (self.ttc < self.FB_Stopping_Time):
                self.AEB_state = "FB"
                self.aeb_status = 1
                self.fcw_status = 1
                self.deacc = self.AEB_FB
            elif self.AEB_state == "PB2" and self.stop_bool is True:
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
            elif self.AEB_state == "FB" and ((self.stop_bool is True) and (abs(self.ttc) > abs(1.2*self.FCW_Stopping_Time))):
                self.AEB_state = "Default"
                self.aeb_status = 0
                self.fcw_status = 0
                self.deacc = 0
        
            return self.deacc, self.aeb_status, self.fcw_status
        else: 
            a3 = "TTC negative! You have crashed! "
            print(a3)
            a2 = 0
            a1 = 0
            return a1, a2, a3
    
        
    def get_controls(self, relative_dist, ego_vel, ACC_set_speed, ttc, ego_acc, driver_brake, dt):
        self.relative_dist = relative_dist
        self.ego_vel = ego_vel
        self.ACC_set_speed = ACC_set_speed
        self.ttc = ttc
        self.ego_acc = ego_acc
        self.ACC_enable = int((ACC_set_speed > 0))
        # perform the functions using the updated attributes
        # other attributes will get updated as the function runs
        self.stop_bool = self.TTC_non_positive()
        self.FCW_Stopping_Time = self.stopping_time()
        self.PB1_Stopping_Time, self.PB2_Stopping_Time, self.FB_Stopping_Time = self.stopping_time_calc()
        acc_acceleration = self.acc_controller()
        aeb_deceleration, aeb_status, fcw_status = self.AEB_state_machine()
        brakecontrol = Brake_Control(acc_acceleration, aeb_deceleration, driver_brake)
        brake_command = brakecontrol.final_decel()
        throttle = Throttle_control(aeb_status, acc_acceleration)
        throttle_command = throttle.switch()
        #ego_acc = throttle_command
        #ego_vel += (throttle_command )*dt
        
        return brake_command, throttle_command
        
    
        
        
    

############################################################################
class Brake_Control:

    def __init__(self, ACC_deacc, deceleration, driver_brake):
        self.ACC_deacc = -1*ACC_deacc
        self.deceleration = deceleration
        self.driver_brake = driver_brake
        self.brake = 2
     
    def final_decel(self):
        if self.ACC_deacc > 0:
            self.ACC_deacc = self.ACC_deacc
        else:
            self.ACC_deacc = 0
        brake_final = max(self.brake, self.ACC_deacc, self.deceleration)
        return brake_final

#######################################################################
class Throttle_control:
    
    def __init__(self, aeb_status, acc):
        self.aeb_status = aeb_status
        self.acc = acc
        self.acc_bool = True
        self.AEB_or_acc_bool = True
        self.throttle = 0

        
    def switch(self):
        
        # check if acceleration is positive or negative
        if self.acc <= 0:
            self.acc_bool = True
        else:
            self.acc_bool = False

        # check the aeb status
        if (self.aeb_status is True) or (self.acc_bool is True):
            self.AEB_or_acc_bool = True
        else:
            self.AEB_or_acc_bool = False

        # return the throttle command
        if (self.AEB_or_acc_bool is True):
            self.throttle = 0
        else:
            self.throttle = self.acc
        return self.throttle
##########################################################################################


    




