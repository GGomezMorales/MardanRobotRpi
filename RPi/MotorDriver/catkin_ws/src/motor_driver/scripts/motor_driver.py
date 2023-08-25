#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

import zumo_lib as motors
import threading

class MockMotorController:
    def __init__(self):
        pass

    def send_speed(self, speed_l, speed_r, log = False):
        print("speed_l =", speed_l, " --- speed_r =", speed_r)

class I2CMotorController:
    def __init__(self):
        self.motors.init_i2c_bus()

    def send_speed(self, speed_l, speed_r, log = False):
        self.motors.send_speed(speed_l, speed_r)

class MotorController:
    def __init__(self, motor_interface):
        self.sub = rospy.Subscriber("motors/motor_twist", Twist, self.data_processing_callback)
        self.vel_max = 200
        self.l_val = 0
        self.r_val = 0
        self.rate = rospy.Rate(10)
        self.worker = threading.Thread(target=self.i2c_thread)
        self.motor_interface = motor_interface

    def data_processing_callback(self, twist):
        self.l_val = int((twist.linear.x - twist.angular.z)*self.vel_max/2)
        self.r_val = int((twist.linear.x + twist.angular.z)*self.vel_max/2)

    def i2c_thread(self):
        while not rospy.is_shutdown():
            motors.send_speed(self.l_val, self.r_val)
            self.rate.sleep()

    def star_worker(self):
        self.worker.start()

def main():
    ## i2cMotorController = I2CMotorController()
    i2cMotorController = MockMotorController()
    rospy.init_node('motor_driver')
    motorController = MotorController(i2cMotorController)
    motorController.star_worker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
