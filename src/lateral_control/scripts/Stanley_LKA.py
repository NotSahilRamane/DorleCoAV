#!/usr/bin/env python3
import math
from turtle import heading
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class Stanley_LKA(object):
    def __init__(self, crosstrack_error, heading_angle_road, car_heading, ego_velocity_longitudinal):
        self.crosstrack_error=crosstrack_error
        self.heading_angle=heading_angle_road
        self.carheading=car_heading
        self.orientation_error=heading_angle_road-car_heading
        self.ego_velocity=ego_velocity_longitudinal
        self.getSteeringAngle()

    def getSteeringAngle(self):
        self.steering= self.orientation_error+np.arctan2(self.k*self.crosstrack_error,self.ego_velocity)
        return self.normalize_angle(self.steering)
        
    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    
