#!/usr/bin/python3

from taohunza2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from pizza_interface.srv import Reach
from std_msgs.msg import Bool,Float64MultiArray,Int16
import numpy as np

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.create_subscription(Pose,'pose',self.pose_callback,10)
        self.create_subscription(Pose,'target',self.target_callback,10)
        self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        self.reach_client = self.create_client(Reach,'reach_notify')
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kp_angular', 10.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.tim = 1.0/self.frequency
        self.create_timer(self.tim,self.timer_callback)

        self.kp = 1.0
        self.kp_angular = 10.0
        self.robot_pose = np.array([0.0,0.0,0.0])
        self.robot_target = np.array([0.0,0.0,0.0])
        self.flag = 0

    def cmdvel(self,v,w):
        msg =  Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel.publish(msg)
        
    def pose_callback(self,msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def target_callback(self,msg):
        self.flag = 1
        self.robot_target[0] = msg.x
        self.robot_target[1] = msg.y
        self.robot_target[2] = msg.theta


    def timer_callback(self):
        if self.flag == 1:
            self.kp = self.get_parameter('kp').get_parameter_value().double_value
            self.kp_a = self.get_parameter('kp_angular').get_parameter_value().double_value
            now_x = self.robot_pose[0]
            now_y = self.robot_pose[1]
            diff_x = self.robot_target[0] - now_x 
            diff_y = self.robot_target[1] - now_y 
            distance = np.sqrt((diff_x**2)+(diff_y**2))
            diff_theta = np.arctan(diff_y/diff_x) - self.robot_pose[2]
            # use_theta = np.arctan2(np.sin(diff_theta),np.cos(diff_theta))
            use_theta = np.arctan2(diff_y,diff_x)  - self.robot_pose[2]
            use_theta = np.arctan2(np.sin(use_theta), np.cos(use_theta))
            wz = self.kp_a*use_theta
            if(distance < 0.1):
                vx = 0.0
                wz = 0.0
                self.flag = 0
                msg = Reach.Request()
                msg.reach.data = True
                self.reach_client.call_async(msg)
            else:
                # vx = self.kp*distance
                vx = (self.kp*distance)+0.8
            self.cmdvel(vx,wz)
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
