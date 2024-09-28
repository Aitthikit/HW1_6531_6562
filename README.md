# HW1_6531_6562

### Architecture
   taohunza package :
    ![Alt text]()
    This package have 2 main package name teleop_twist_keyboard and turtlesim_plus.
   
    The teleop_twist_keyboard package use for recieve input from you keyboard.

    The turtlesim_plus package use for interface and spawn turtle.

    This package have 4 Node :

        copy_turtle_node : This node for control copy turtle.
   
        pizza_memory_node : This node for collect position data of pizza from turtle. 

        turtle_teleop_node : This node for control turtle.

        Melodic_turtle_node : This node for control Melodic turtle.


### 1. Install Environment
1. Install Ubuntu 22.04
   
    https://ubuntu.com/download/desktop

2. Install ROS2 (Humble)

   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 2. Install Package
1. Clone taohunza2 package
   ```sh
   git clone https://github.com/Aitthikit/HW1_6531_6562.git
   ```

### 3. Setup Environment
1. Navigate to the project directory
   ```sh
    cd HW1_6531_6562
    ```
2. Remove Build install and log Before build:
    ```sh
    rm -rf build/ install/ log/
    ```
3. Install pynput :
    ```sh
    pip install pynput
    ```
4. Build :
    ```sh
    colon build
    ``` 
5. Source workspace :
    ```sh
    source install/setup.bash 

### 4. Run
1. Run launch file :
   ```sh
   ros2 launch taohunza2 turtle_launch.py 
   ```
2. Run teleop_key :
   ```sh
   ros2 run taohunza2 teleop_key.py
   ``` 
4. Run rqt :
   ```sh
   rqt
   ```


## How to Fix when Bug
1. If it cannot run node or launch file:
   
    1.1. Restart your terminal
   
    2.2. Rebuild & source
    ```sh
    colon build
    source install/setup.bash
    ```
2. Turtle can't eat pizza :
   
   1.1. Tune gain (Recomend kp : 1 , kp_angular : 10)
   
4. Something else :
   
   2.1. Kill all node
   
   2.2. Restart your computer
   
   2.3. Ask chatGPT and let's fix it
   2.4. 

