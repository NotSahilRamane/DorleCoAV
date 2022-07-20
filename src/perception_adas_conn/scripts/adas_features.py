#!/usr/bin/env python3

import rospy

from av_messages.msg import object
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ADAS_Features:
    def __init__(self):
        self.loadParameters()
        # self.bridge = CvBridge()


    def subscribeToTopics(self):
        rospy.loginfo('Subscribed to topics')
        rospy.Subscriber(self.MOT_object_topicname, object,
                        self.extractDataMOO, queue_size=1)
        rospy.Subscriber(self.road_seg_topicname, Twist,
                        self.callback1, queue_size=1)
        rospy.Subscriber(self.ego_veh_velocity_topicname, Odometry,
                        self.callback2, queue_size=1)

    def loadParameters(self):
        self.MOT_object_topicname = rospy.get_param("")
        self.road_seg_topicname = rospy.get_param("")
        self.ego_veh_velocity_topicname = rospy.get_param("")
        self.pub_topic_name = rospy.get_param("") 
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.ControlsPublisher = rospy.Publisher(
            self.pub_topic_name, Twist, queue_size=1)
        
    def extractDataMOO(self, obj_data):
        self.MOT_velocity = obj_data.object_state_dt
        self.MOT_position = obj_data.position

    def extractDataRoadSeg(self, seg_data):
        self.go_flag = seg_data.linear.x
        self.distance = seg_data.linear.y

    def extractEgoVehVelocity(self, odometry):
        self.ego_velocity_x = odometry.twist.linear.x
        self.ego_velocity_y = odometry.twist.linear.y
        self.ego_velocity_angular = odometry.angular.z

    def Algorithm(self):
        # inputs and parameters before simulation starts 
        end_time = 10             # set simulation end time
        dt = 0.1                    # set the resolution    
        error = numpy.zeros(100)
        error_y = numpy.zeros(100)
        initial_rel_dist = min(extractDataMOO.MOT_position,extractDataRoadSeg.distance)
        initial_ego_vel = extractEgoVehVelocity.ego_velocity_x
        initial_acc_set_speed = 20
        initial_ttc = initial_rel_dist/initial_ego_vel
        initial_ego_acc = 1

        # initialise an object at the beginning of the simulation. 
        # same object keeps on updating as the simulation runs          
        aeb = AEB_Controller(initial_rel_dist, initial_ego_vel, initial_acc_set_speed, initial_ttc, initial_ego_acc)
        ego_vel = initial_ego_vel
        ego_acc = initial_ego_acc
        # all the code below goes in a for or while loop with iterator as "current_time"
        ############
        for current_time in range(100):
            #print(current_time)
            # define the inputs to be taken from perception module 
            aeb.MOO = aeb.MOO_Dist_Calc()
            rel_dist = aeb.MOO

            #ego_vel = None 
            acc_set_speed = 20 
            #ttc = 50 - current_time*dt 
            #ego_acc = None 
            driver_brake = 0 

            print(rel_dist, ttc)


            # update the aeb attributes using the above inputs 
            aeb.relative_dist = rel_dist
            aeb.ego_vel = ego_vel
            aeb.ACC_set_speed = acc_set_speed
            aeb.ttc = rel_dist/ego_vel
            aeb.ego_acc = ego_acc  

            # perform the functions using the updated attributes
            # other attributes will get updated as the function runs
            aeb.stop_bool = aeb.TTC_non_positive()
            aeb.FCW_Stopping_Time = aeb.stopping_time()
            aeb.PB1_Stopping_Time, aeb.PB2_Stopping_Time, aeb.FB_Stopping_Time = aeb.stopping_time_calc()
            acc_acceleration = aeb.acc_controller(current_time)
            aeb_deceleration, aeb_status, fcw_status = aeb.AEB_state_machine()

            brakecontrol = Brake_Control(acc_acceleration, aeb_deceleration, driver_brake)
            brake_command = brakecontrol.final_decel()
            throttle = Throttle_control(aeb_status, acc_acceleration)
            throttle_command = throttle.switch()
            ego_acc = throttle_command
            ego_vel += (throttle_command )*dt

            print("brake = {}".format(brake_command))
            print("throttle = {}".format(throttle_command))
        controls = 1.0 # just for the time being
        self.callPublisher(controls)

    def callPublisher(self, controls):
        self.ControlsPublisher.publish(controls)

# object - velocity, pos
# road-seg - geometry/twist
# vehicle_velot - odometry
