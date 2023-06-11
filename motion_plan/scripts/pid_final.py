#!/usr/bin/env python3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Quaternion, Pose, Twist
import rospy
from sensor_msgs.msg import LaserScan, Imu , PointCloud2
from tf.transformations import euler_from_quaternion
from laser_geometry import LaserProjection
import math
import numpy as np
pub = None

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp      #proportional constant
        self.ki = ki      #integral constant
        self.kd = kd      #differential constant
        self.fw_dist = 0      #desired distance from wall
        self.c_dist = 0      #current distance from wall
        self.Error = 0       #current error
        self.kiError = 0     #error sum
        self.prevError = 0   #previous error
    
    def control(self, w_dist, c_dist):
        self.w_dist = w_dist
        self.c_dist = c_dist
        self.Error = self.w_dist - self.c_dist         #error calculation
        self.Proportional = self.kp * self.Error       #proportional out 
        self.kiError = self.kiError + self.Error       #total error calculation
        self.Integral = self.ki * self.kiError         #integral out    
        self.Differential = self.kd * (self.Error*self.prevError)   #differential out
        self.prevError = self.Error

        self.Total_out =  self.Proportional+self.Integral+self.Differential

        return self.Total_out
    

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    
    take_action(regions)

def take_action(regions):
    global diff
    obj = PID(0.5, 0.005, 0.2)
    msg = Twist()
    w_dist = regions['left']
    c_dist = regions['right']

    Total_out = obj.control(w_dist, c_dist)

    f_dist = regions['front']
    if(f_dist<1):
        diff = obj.control(2, f_dist)
    else:
        diff = 0
    linear_x = 0.5 -diff
    angular_z = Total_out
    msg.linear.x = linear_x
    msg.angular.z = -angular_z
    pub.publish(msg)
class ImuToOdom:
    def __init__(self):
       
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.pose = Pose()
        self.twist = Twist()
        self.orientation = Quaternion()

    def imu_callback(self, msg):
        # Get orientation and angular velocity from IMU
        self.orientation = msg.orientation
        angular_velocity = msg.angular_velocity

        # Convert orientation to euler angles
        roll, pitch, yaw = self.quat_to_euler(self.orientation)

        # Update twist message
        self.twist.linear = Vector3(0, 0, 0)
        self.twist.angular = angular_velocity

        # Update pose message
        self.pose.position = Vector3(0, 0, 0)
        self.pose.orientation = self.orientation

        #  publish Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom_frame"
        odom.child_frame_id = "base_link"
        odom.pose.pose = self.pose
        odom.twist.twist = self.twist

        self.odom_pub.publish(odom)

    def quat_to_euler(self, quat):
        # Convert quaternion to euler angles
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

class LaserToCloud:
    def __init__(self):
        self.laser_proj = LaserProjection()
        self.pc_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=1)
        self.laser_sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, self.laserscan_callback)

    def laserscan_callback(self, msg):
        
        cloud_out = self.laser_proj.projectLaser(msg)
        cloud_out.header.frame_id = msg.header.frame_id
        self.pc_pub.publish(cloud_out)

def main():
    global pub
    rospy.init_node('wall_centering')   
    rate = rospy.Rate(10) 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
    sub_laser = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)    
    imu_to_odom = ImuToOdom()
    ltp = LaserToCloud()
    rospy.spin()

if __name__ == '__main__':
    main()
