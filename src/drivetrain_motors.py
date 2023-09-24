#!/usr/bin/env python3

from threading import Lock
from time import time
from time import sleep
import os
import random

import rospy

from std_msgs.msg import Float32MultiArray

from myactuator_control.myactuator_lib import MyActuatorMotor

########################## 
USE_CAN = True
# USE_CAN = False
########################## 


def main():    
    myactuator = MyActuator()
    try:
        myactuator.loop()
    except Exception as e:
        # We really want to make sure that we close the serial port
        # under any circumstance, So catch all errors here
        rospy.logerr(e)
    
    # Set CAN Interface connection down so that it will reconnect later
    print("Disconnecting CAN0....")
    os.system("sudo -S /sbin/ip link set can0 down")    

def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(value, lower))

class MyActuator:
    def __init__(self):
        rospy.init_node("roboteq_control")
        self.rate = rospy.Rate(30)

        # read launch params
        self.name = rospy.get_param("~name")
        self.motor_left_index = rospy.get_param("~motor_left_index")
        self.motor_right_index = rospy.get_param("~motor_right_index")
        ###########################################################
        #  Added new params for IDs and motor type
        self.motor_LF_ID = int(rospy.get_param("~motor_LF_ID"), 16)    # CAN ID 0x141
        self.motor_LB_ID = int(rospy.get_param("~motor_LB_ID"), 16)    # CAN ID 0x144
        self.motor_RF_ID = int(rospy.get_param("~motor_RF_ID"), 16)    # CAN ID 0x142
        self.motor_RB_ID = int(rospy.get_param("~motor_RB_ID"), 16)    # CAN ID 0x143
        # Get Acceleration and Deceleration from YAML
        self.accel = rospy.get_param("~acceleration")
        self.decel = rospy.get_param("~deceleration")
        # Get PID Params from YAML
        self.curr_KP = rospy.get_param("~PID_params/curr_KP")
        self.curr_KI = rospy.get_param("~PID_params/curr_KI")
        self.speed_KP = rospy.get_param("~PID_params/speed_KP")
        self.speed_KI = rospy.get_param("~PID_params/speed_KI")
        self.pos_KP = rospy.get_param("~PID_params/pos_KP")
        self.pos_KI = rospy.get_param("~PID_params/pos_KI")
        # Get params for motr type
        self.motor_type_1 = rospy.get_param("~motor_type_1")
        self.motor_type_2 = rospy.get_param("~motor_type_2")
        # Get params for motor port
        self.motor_LF_port = rospy.get_param("~motor_LF_port")
        self.motor_LB_port = rospy.get_param("~motor_LB_port")
        self.motor_RF_port = rospy.get_param("~motor_RF_port")
        self.motor_RB_port = rospy.get_param("~motor_RB_port")
        # Create motor objects
        self.motor_LF = MyActuatorMotor(self.motor_type_2, self.motor_LF_ID, USE_CAN, self.motor_LF_port) #0x141
        self.motor_LB = MyActuatorMotor(self.motor_type_2, self.motor_LB_ID, USE_CAN, self.motor_LB_port) #0x144
        self.motor_RF = MyActuatorMotor(self.motor_type_2, self.motor_RF_ID, USE_CAN, self.motor_RF_port) #0x142
        self.motor_RB = MyActuatorMotor(self.motor_type_1, self.motor_RB_ID, USE_CAN, self.motor_RB_port) #0x143

        # Just in case.
        self.stop_all()

        # Write KP and KI values to Motors (From PID Params)
        pid_values = self.motor_LB.pid_params_to_value(self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        self.motor_LF.write_pid("ROM", pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])
        self.motor_LB.write_pid("ROM", pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])
        self.motor_RF.write_pid("ROM", pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])
        self.motor_RB.write_pid("ROM", pid_values[0], pid_values[1], pid_values[2], pid_values[3], pid_values[4], pid_values[5])
        # self.motor_LF.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        # self.motor_LB.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        # self.motor_RF.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        # self.motor_RB.write_pid("ROM",self.curr_KP,self.curr_KI,self.speed_KP,self.speed_KI,self.pos_KP,self.pos_KI)
        
        # Write Accelerations/Decelerations from Yaml file to motors
        self.motor_LF.write_acceleration(0x02, self.accel)    # Write Speed Control Acceleration
        self.motor_LF.write_acceleration(0x03, self.decel)    # Write Speed Control Deceleration
        self.motor_LB.write_acceleration(0x02, self.accel)    # Write Speed Control Acceleration
        self.motor_LB.write_acceleration(0x03, self.decel)    # Write Speed Control Deceleration
        self.motor_RF.write_acceleration(0x02, self.accel)    # Write Speed Control Acceleration
        self.motor_RF.write_acceleration(0x03, self.decel)    # Write Speed Control Deceleration
        self.motor_RB.write_acceleration(0x02, 20000)    # Write Speed Control Acceleration
        self.motor_RB.write_acceleration(0x03, 20000)    # Write Speed Control Deceleration
        ############################################################
        ccw_fwd = rospy.get_param("~ccw_fwd")
        self.timeout = rospy.get_param("~timeout")
        
        # Set the ccw_correct for both left and right wheels, which should be opposite of each other
        self.ccw_correct_left = 1 if ccw_fwd else -1
        self.ccw_correct_right = -self.ccw_correct_left

        # create publishers and subscribers
        motor_cmd_topic = rospy.get_param("~motor_cmd_topic")
        #myactuator_data_topic = rospy.get_param("~myactuator_data_topic")
        self.motor_cmd_sub = rospy.Subscriber(motor_cmd_topic, Float32MultiArray, self.motor_cmd_callback, queue_size=1)
        #self.data_pub = rospy.Publisher(myactuator_data_topic, MyActuatorData, queue_size=1)    
        self.current_pub = rospy.Publisher("drivetrain_current_data", Float32MultiArray, queue_size=1)

        # boundary on values acceptable to write to the motors (abs val)
        self.max_motor_val = 600.0        ############# Updated Max speed 
        
        self.last_tick = time()

        # Make a dictonary to store motor current
        self.current_dict = {"lf": 0, "lb": 0, "rf": 0, "rb": 0}


    def motor_val(self, unit_motor_speed: float, ccw_correction:int) -> int:
        clamped_motor_speed = clamp(unit_motor_speed, -1.0, 1.0)
        motor_val = int(clamped_motor_speed * self.max_motor_val * ccw_correction)
        return motor_val


    def motor_cmd_callback(self, msg: Float32MultiArray):
        self.last_tick = time()
        # read unit motor speeds from msg
        motor_left_unit_speed = msg.data[self.motor_left_index]
        motor_right_unit_speed = msg.data[self.motor_right_index]
        #print("Motor callback")
        # translate unit motor speeds to values motors will accept
        motor_left_val = self.motor_val(motor_left_unit_speed, self.ccw_correct_left)        # Left motor command
        motor_right_val = self.motor_val(motor_right_unit_speed, self.ccw_correct_right)    # Right motor command

        # Send speed commands to the motors and save the electrical current values to the dictionary 
        self.current_dict["lf"] = self.motor_LF.speed_control(-motor_left_val, read=True)
        self.current_dict["lb"] = self.motor_LB.speed_control(motor_left_val, read=True)
        self.current_dict["rf"] = self.motor_RF.speed_control(motor_right_val, read=True)
        self.current_dict["rb"] = self.motor_RB.speed_control(motor_right_val, read=True)
        self.send_currents()

    def send_currents(self):
        current_msg = Float32MultiArray()
        current_msg.data = [
            self.current_dict.get("lf", 0),
            self.current_dict.get("lb", 0),
            self.current_dict.get("rf", 0),
            self.current_dict.get("rb", 0)
        ]

        self.current_pub.publish(current_msg)

    def stop_all(self):
        self.motor_LF.stop()
        self.motor_LB.stop()
        self.motor_RF.stop()
        self.motor_RB.stop()


    def loop(self):
        #print("Loop start")
        while not rospy.is_shutdown():
            self.rate.sleep()
            # tell robot to stay still if connection lost
            timed_out = (time() - self.last_tick) > self.timeout
            if timed_out:
                print("Timeout")
                self.stop_all()


if __name__ == "__main__":
    main()
