#!/usr/bin/python3

from taohunza2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.srv import Spawn
from std_srvs.srv import Empty
from pizza_interface.srv import Reach,Run,Make,Save,Clear
from std_msgs.msg import Bool,Float64MultiArray,Int16,Int64
import numpy as np
import yaml
import os


class TeleopSchedulerNode(Node):
    def __init__(self):
        super().__init__('teleop_schduler_node')
        self.declare_parameter('pizza_positionX',[0.0])
        self.declare_parameter('pizza_positionY',[0.0])
        self.yaml_file_path = os.path.join(os.getcwd(), 'src/taohunza2/config/pizzapath.yaml')
        self.create_subscription(Twist,'/custom_key/cmd_vel',self.cmd_vel_callback,10)
        self.create_subscription(Pose,'pose',self.pose_callback,10)
        self.create_subscription(Int64,'pizza_count',self.pizza_callback,10)
        self.cmd_vel = self.create_publisher(Twist,'cmd_vel',10)
        self.target_pub = self.create_publisher(Pose,'target',10)
        # self.Run_server = self.create_service(Run,'/run',self.run_callback)
        self.create_service(Make,'/make',self.make_callback)
        self.create_service(Save,'/save',self.save_callback)
        self.create_service(Clear,'/clear',self.clear_callback)
        self.create_service(Reach,'reach_notify',self.reach_callback)

        self.spawn_client = self.create_client(GivePosition,'/field1/spawn_pizza')
        self.runcopy_client = self.create_client(Run,'/run')
        self.eat_pizza_client = self.create_client(Empty,'eat')
        self.declare_parameter('frequency', 10.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.tim = 1.0/self.frequency
        self.create_timer(self.tim,self.timer_callback)

        self.flag = 0
        self.keyvel = Twist()
        self.robot_pose = np.array([0.0,0.0,0.0])
        self.pizza_amount = 0
        self.pizza_now = 0
        self.maxpizza = 0
        self.mem = 0
        self.pizza_positionX = []
        self.pizza_positionY = []
        self.declare_parameter('max_pizza', 100.0)
        self.maxpizza = self.get_parameter('max_pizza').get_parameter_value().double_value


    def cmd_vel_callback(self,msg):
        self.keyvel = msg
    def pose_callback(self,msg:Pose):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta
    def pizza_callback(self,msg:Int64):
        self.pizza_now = msg.data   
    def spawn_pizza(self,x,y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.pizza_amount += 1
        self.spawn_client.call_async(position_request)
    def reach_callback(self, request:Reach.Request , response:Reach.Response):
        if request.reach.data:
            self.eat_pizza()
            self.pizza_amount -= 1
            self.flag = 3
            self.pizza_positionX.pop(0)
            self.pizza_positionY.pop(0)
            if len(self.pizza_positionX) != 0:
                msg = Pose()
                msg.x = self.pizza_positionX[0]
                msg.y = self.pizza_positionY[0]
                self.target_pub.publish(msg)
            else:
                self.flag = 0
        return response
    
    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def make_callback(self, request:Make.Request , response:Make.Response):
        self.maxpizza = self.get_parameter('max_pizza').get_parameter_value().double_value
        if request.make.data:
            if self.pizza_amount < self.maxpizza:
                self.spawn_pizza(self.robot_pose[0],self.robot_pose[1])
                self.pizza_positionX.append(self.robot_pose[0])
                self.pizza_positionY.append(self.robot_pose[1])
                self.get_logger().info(f'Now pizza = {self.pizza_amount}/{self.maxpizza}')
            else:
                self.get_logger().info(f'NO more pizza = {self.pizza_amount}/{self.maxpizza}')
        return response
    
    def clear_callback(self, request:Clear.Request , response:Clear.Response):
        if request.clear.data:
            # self.eat_pizza()
            if self.flag == 0:
                self.flag = 1
            else:
                self.get_logger().info(f'Wait for another process')
        return response
    
    def save_callback(self, request:Save.Request , response:Save.Response):
        if request.save.data:
            if self.flag == 0:
                if self.mem < 4:
                    self.flag = 2
                else:
                    self.get_logger().info(f'Please reset!!!!')    
            else:
                self.get_logger().info(f'Wait for another process')
        return response
    
    

    def pizza_positionX_mem(self,msg):
        # self.get_logger().info(f'posX {self.state}')
        msg = list(map(float, msg))
        if self.mem == 0:
            self.update_yaml_file(f'/field2/Foxy','pizza_positionX', msg)
            self.melodic_posX = msg
        elif self.mem == 1:
            self.update_yaml_file(f'/field2/Noetic','pizza_positionX', msg)
            self.melodic_posX += msg
        elif self.mem == 2:
            self.update_yaml_file(f'/field2/Humble','pizza_positionX', msg)
            self.melodic_posX += msg
        elif self.mem == 3:
            self.update_yaml_file(f'/field2/Iron','pizza_positionX', msg)
            self.melodic_posX += msg
            self.update_yaml_file(f'/field2/Melodic','pizza_positionX', self.melodic_posX )

    def pizza_positionY_mem(self,msg):
        # self.get_logger().info(f'posY {msg}')
        msg = list(map(float, msg))
        if self.mem == 0:
            self.update_yaml_file(f'/field2/Foxy','pizza_positionY', msg)
            self.melodic_posY = msg
        elif self.mem == 1:
            self.update_yaml_file(f'/field2/Noetic','pizza_positionY', msg)
            self.melodic_posY += msg
        elif self.mem == 2:
            self.update_yaml_file(f'/field2/Humble','pizza_positionY', msg)
            self.melodic_posY += msg
        elif self.mem == 3:
            self.update_yaml_file(f'/field2/Iron','pizza_positionY', msg)
            self.melodic_posY += msg
            self.update_yaml_file(f'/field2/Melodic','pizza_positionY', self.melodic_posY )


    def update_yaml_file(self, namespace, param_name, param_value):
        yaml_file_path = os.path.join(os.getcwd(), 'src/taohunza2/config/pizzapath.yaml')

        # Load existing YAML data
        if os.path.exists(yaml_file_path):
            with open(yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file) or {}
        else:
            yaml_data = {}

        # Split the namespace into components (e.g., 'field2/melodic' -> ['field2', 'melodic'])
        namespace_parts = namespace.strip('/').split('/')

        # Traverse the namespace hierarchy in the YAML data
        current_level = yaml_data
        for part in namespace_parts:
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        # Ensure the ros__parameters exists at the correct level
        if 'ros__parameters' not in current_level:
            current_level['ros__parameters'] = {}

        # Update the specific parameter
        current_level['ros__parameters'][param_name] = param_value

        # Write back to the YAML file
        with open(yaml_file_path, 'w') as file:
            yaml.dump(yaml_data, file, default_flow_style=False)

        print(f"Updated {param_name} under {namespace} in {yaml_file_path}")
        # # Load the YAML file
        # with open(self.yaml_file_path, 'r') as file:
        #     yaml_data = yaml.safe_load(file)
        # # Update the parameter value
        # if node_name in yaml_data:
        #     yaml_data[node_name]['ros__parameters'][param_name] = param_value
        # else:
        #     # If the node doesn't exist, create it
        #     yaml_data[node_name] = {
        #         'ros__parameters': {
        #             param_name: param_value
        #         }
        #     }
        # # Write the updated YAML data back to the file
        # with open(self.yaml_file_path, 'w') as file:
        #     yaml.dump(yaml_data, file, default_flow_style=False)
        # self.get_logger().info(f"Updated {param_name} to {param_value} in {self.yaml_file_path}")


    def timer_callback(self):
        if self.flag == 1:
            if len(self.pizza_positionX) == 0:
                self.flag = 0
            else: 
                msg = Pose()
                msg.x = self.pizza_positionX[0]
                msg.y = self.pizza_positionY[0]
                self.target_pub.publish(msg)
        elif self.flag == 2:
            self.pizza_positionX_mem(self.pizza_positionX)
            self.pizza_positionY_mem(self.pizza_positionY)
            self.pizza_positionX.clear()
            self.pizza_positionY.clear()
            self.mem += 1
            if self.mem == 4:
                msg = Run.Request()
                msg.run.data = True
                self.runcopy_client.call_async(msg)
            self.flag = 0
        elif self.flag == 3:
            pass
        else:
            # self.get_logger().info(f'Make {self.keyvel}')
            self.cmd_vel.publish(self.keyvel)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
