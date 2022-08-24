#!/usr/bin/env python3
from cmath import inf
import math
from threading import Timer
import numpy
import scipy
import pandas
import matplotlib.pyplot as plt


class LCA_Control:

    def __init__(self, LaneDetections, egoCar, activate, LCA_Signal):
        self.LaneDetections = LaneDetections
        self.egoCar = egoCar
        self.activate = activate  # add logic for rising edge detection
        self.direction = LCA_Signal
        self.LW = 3.7
        self.half_lane_width_estimate = 1.5
        self.Center_Curvature = 0
        self.Center_Curvature_Derivative = 0
        self.Center_Heading_Angle = 0
        self.Center_Lateral_Offset = 0

    def LCA_func(self):
        self.HE = self.Center_Heading_Angle
        self.LE = -self.Center_Lateral_Offset
        curvature = self.Center_Curvature
        dt = 0.1
        arraysize = 30/dt
        x = numpy.zeros(arraysize, 1)
        y = numpy.zeros(arraysize, 1)
        theta = numpy.zeros(arraysize, 1)
        path = [0, 0, 0]
        V = 0
        
        if not path1:
            path1 = [x, y, theta]

        if not ego_start:
            ego_start = [0, 0, 0]

        if not time_start:
            time_start = 0

        if self.activate == 1:
            #Polynomial in frenet
            ego_start = [self.egoCar.Position(
                1), self.egoCar.Position(2), self.egoCar.Yaw]
            time_start = self.cur_time
            V = math.sqrt(self.egoCar.Velocity(1) ^ 2 +
                          self.egoCar.Velocity(2) ^ 2)
            D = V*1
            if curvature < 0.00001:
                curvature = 0.00001

            R = 1/curvature
            M = [[1, 0, 0, 0, 0, 0],
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 1, 0, 0, 0],
                 [1, D, D**2, D**3, D**4, D**5],
                 [0, 1, 2*D, 3*(D**2), 4*(D**3), 5*D**4],
                 [0, 0, 2, 6*D, 12*(D**2), 20*(D**3)]]
            alpha = 0
            for i in range(arraysize):
                t = i*dt
                # Frenet
                x_f = V*t
                y_f = 0
                if self.direction == 1:  # Left Lane change Path
                    y_1 = [[0], [math.tan(self.HE)], [0],
                           [- self.LE + self.LW], [0], [0]]
                    y1 = numpy.reshape(y_1, (6, 1))
                    lon = M/y1  
                    if (x_f < D):
                        y_f = lon(1) + lon(2)*(x_f) + lon(3)*(x_f) ^ 2 + lon(4) * \
                            (x_f) ^ 3 + lon(5) * \
                            (x_f) ^ 4 + lon(6)*(x_f) ^ 5
                    else:
                        y_f = -self.LE + self.LW

                elif self.direction == 2:  # Right Lane change Path
                    y1 = [[0], [math.tan(self.HE)], [0],
                          [-self.LE - self.LW], [0], [0]]
                    lon = M/y1  
                    if (x_f < D):
                        y_f = lon(1) + lon(2)*(x_f) + lon(3)*(x_f) ^ 2 + \
                            lon(4)*(x_f) ^ 3 + lon(5) * \
                            (x_f) ^ 4 + lon(6)*(x_f) ^ 5
                    else:
                        y_f = -self.LE - self.LW

                # Tranform to Cartesian
                alpha = alpha + V*dt/(R-y_f)
                #alpha = alpha + V*dt/(R)
                x1 = (R-y_f)*math.sin(alpha)
                y1 = R-(R-y_f)*math.cos(alpha)
                Rot_1 = [math.cos(self.HE), -math.sin(self.HE),
                         math.sin(self.HE), math.cos(self.HE)]
                Rot = numpy.reshape(Rot_1, (2, 2))
                rot_vec = Rot*[x1, y1]
                x[i] = rot_vec(1)
                y[i] = rot_vec(2)

            path1 = [x, y, theta]

        i = round((self.cur_time-time_start)/dt+1)
        path = path1[i]
        cur_pose = [self.egoCar.Position(1), self.egoCar.Position(
            2), self.egoCar.Yaw] - ego_start
        long_vel = V

        return path, cur_pose, long_vel, self.activate #first three are inputs for the Stanley controller code

    def lane_detection_selection(self):
        if self.LaneDetections.left.Strength > 0 and self.LaneDetections.right.Strength > 0:
            self.Center_Curvature = (
                self.LaneDetections.left.curvature + self.LaneDetections.right.curvature)/2
            self.Center_Curvature_Derivative = (
                self.LaneDetections.left.curvature_derivative+self.LaneDetections.right.curvature_derivative)/2
            self.Center_Heading_Angle = (
                self.LaneDetections.left.heading_angle+self.LaneDetections.right.heading_angle)/2
            self.Center_Lateral_Offset = (
                self.LaneDetections.left.lateral_offset+self.LaneDetections.right.lateral_offset)/2

        elif self.LaneDetections.left.Strength > 0:
            self.Center_Curvature = (
                self.LaneDetections.left.curvature)/(1-(1.5*self.LaneDetections.left.curvature))
            self.Center_Curvature_Derivative = (
                self.LaneDetections.left.curvature_derivative)/((1-(1.5*self.LaneDetections.left.curvature))**2)
            self.Center_Heading_Angle = self.LaneDetections.left.heading_angle
            self.Center_Lateral_Offset = self.LaneDetections.left.lateral_offset-1.5

        elif self.LaneDetections.right.Strength > 0:
            self.Center_Curvature = (
                self.LaneDetections.right.curvature)/(1+(1.5*self.LaneDetections.right.curvature))
            self.Center_Curvature_Derivative = (
                self.LaneDetections.right.curvature_derivative)/((1+(1.5*self.LaneDetections.right.curvature))**2)
            self.Center_Heading_Angle = self.LaneDetections.right.heading_angle
            self.Center_Lateral_Offset = self.LaneDetections.right.lateral_offset-1.5

        return self.Center_Curvature, self.Center_Curvature_Derivative, self.Center_Heading_Angle, self.Center_Lateral_Offset
