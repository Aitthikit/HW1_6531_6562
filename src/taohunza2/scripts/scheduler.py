#!/usr/bin/python3

from taohunza2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from pizza_interface.srv import Reach,Run,Make,Save,Clear,Emptysto
from std_msgs.msg import Bool,Float64MultiArray,Int16,Int64
import numpy as np
import yaml
import os

class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        # Path to the YAML file
        self.yaml_file_path = os.path.join(os.getcwd(), 'src/taohunza2/config/pizzapath.yaml')
        self.create_subscription(Pose,'pose',self.pose_callback,10)
        self.create_service(Run,'/run',self.run_callback)
        self.create_service(Emptysto,'/empty_notify',self.empty_callback)
        self.create_service(Reach,'reach_notify',self.reach_callback)
        self.spawn_client = self.create_client(GivePosition,'/field2/spawn_pizza')
        self.target_pub = self.create_publisher(Pose,'target',10)
        self.isempty = self.create_client(Emptysto,'/empty_notify')
        
        self.pizza_positionX = []
        self.pizza_positionY = []
        self.robot_pose = np.array([0.0,0.0,0.0])
        self.declare_parameter('frequency', 10.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.tim = 1.0/self.frequency
        self.create_timer(self.tim,self.timer_callback)
        self.flag = 0
        self.count = 0

    def reach_callback(self,request:Reach.Request , response:Reach.Response):
        if request.reach.data:
            if len(self.pizza_positionX) != 0:
                self.spawn_pizza(self.pizza_positionX[0],self.pizza_positionY[0])
                self.flag = 3
                self.pizza_positionX.pop(0)
                self.pizza_positionY.pop(0)
            if len(self.pizza_positionX) != 0:
                msg = Pose()
                msg.x = self.pizza_positionX[0]
                msg.y = self.pizza_positionY[0]
                self.target_pub.publish(msg)
            else:
                msg2 = Emptysto.Request()
                msg2.empty.data = True
                self.isempty.call_async(msg2)
                self.flag = 0
        return response
    def empty_callback(self,request:Emptysto.Request , response:Emptysto.Response):
        if request.empty.data:
            self.count += 1
        if self.count == 4:
            self.flag = 2
        return response
    def pose_callback(self,msg:Pose):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def spawn_pizza(self,x,y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_client.call_async(position_request)
    
    def run_callback(self,request:Run.Request , response:Run.Response):
        if request.run.data:
            current_namespace = self.get_namespace().strip('/')
            self.get_logger().info(f'{current_namespace}')
                # Load YAML data
            with open(self.yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)

            namespace_parts = current_namespace.split('/')
            if len(namespace_parts) >= 2:
                field_name = namespace_parts[0]  # e.g., field2
                melodic_name = namespace_parts[1]  # e.g., Foxy

                # Check if the field and melodic names exist in the YAML data
                if field_name in yaml_data and melodic_name in yaml_data[field_name]:
                    self.pizza_positionX = yaml_data[field_name][melodic_name]['ros__parameters']['pizza_positionX']
                    self.pizza_positionY = yaml_data[field_name][melodic_name]['ros__parameters']['pizza_positionY']
                    self.flag = 1
            # if current_namespace in yaml_data:
            #     self.pizza_positionX = yaml_data[current_namespace]['ros__parameters']['pizza_positionX']
            #     self.pizza_positionY = yaml_data[current_namespace]['ros__parameters']['pizza_positionY']
                else:
                    self.get_logger().error(f'Namespace {current_namespace} not found in YAML.')

            self.get_logger().info(f'Pizza X positions for {current_namespace}: {self.pizza_positionX}')
            self.get_logger().info(f'Pizza Y positions for {current_namespace}: {self.pizza_positionY}')
        return response
    def timer_callback(self):
        if self.flag == 1:
            if len(self.pizza_positionX) == 0:
                self.flag = 2
            else: 
                msg = Pose()
                msg.x = self.pizza_positionX[0]
                msg.y = self.pizza_positionY[0]
                self.target_pub.publish(msg)
        if self.flag == 2:
            msg = Pose()
            msg.x = 10.0
            msg.y = 10.0
            self.target_pub.publish(msg)
            self.flag = 0
        else:
            pass

    # Access specific node's parameters using the namespace (e.g., field2/melodic)
        

def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
