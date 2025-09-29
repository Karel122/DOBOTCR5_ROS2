# DOBOTCR5_ROS2

# NAVODILA

Ta postopek opisuje kako krmiliti DOBOT-a CR5 z uprabo ROS2 Humble okolja.

# 1. Vzpostavitev povezave z dobotom:
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

Ta ukaz deluje le, če je robot povezan s pravilnim IP-jem

# 2. Zaženi MoveIt:
ros2 launch dobot_moveit dobot_moveit.launch.py

# 3. PowerON:
ros2 service call /dobot_bringup_ros2/srv/PowerOn dobot_msgs_v4/srv/PowerOn "{}"

# 4. Enable the robot:
ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot "{}"

# 5. Disable the robot:
ros2 service call /dobot_bringup_ros2/srv/DisableRobot dobot_msgs_v4/srv/DisableRobot

# 6. Clear Error:
ros2 service call /dobot_bringup_ros2/srv/ClearError dobot_msgs_v4/srv/ClearError "{}"

# 7. Get error ID:
ros2 service call /dobot_bringup_ros2/srv/GetErrorID dobot_msgs_v4/srv/GetErrorID "{}"

# 8. Get angle:
ros2 service call dobot_bringup_ros2/srv/GetAngle dobot_msgs_v4/srv/GetAngle

# 9. Start Drag:
ros2 service call /dobot_bringup_ros2/srv/StartDrag dobot_msgs_v4/srv/StartDrag

# 10. Stop Drag:
ros2 service call /dobot_bringup_ros2/srv/StopDrag dobot_msgs_v4/srv/StopDrag

# 11. MoveIt:
ros2 launch dobot_moveit dobot_moveit.launch.py

# 12. Pozicija:
/dobot_msgs_v4/msg/ToolVectorActual

# 13. Run GazeboSim:
ros2 launch dobot_gazebo dobot_gazebo.launch.py

# move_demo:
ros2 action send_goal --feedback --print-result \
/cr5_group_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
"{ trajectory: {
    joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [
      { positions: [0.0,-0.8,1.0,-1.2,0.8,0.0], time_from_start: {sec: 2} },
      { positions: [0.2,-1.0,1.2,-1.4,1.0,0.1], time_from_start: {sec: 5} }
    ] } }"

    **ROS2_V4 Installation and Configuration Guide**  

---

### **Introduction**  
DOBOT_6Axis-ROS2_V4 is a ROS Software Development Kit (SDK) developed by Dobot based on the TCP/IP protocol. This kit is built using ROS/C++ and Python, adhering to the Dobot-TCP-IP control communication protocol. It establishes a TCP connection with the robotic arm terminal via Socket and provides user-friendly API interfaces. Users can quickly connect to Dobot robotic arms for secondary development and control.  

---

### **Prerequisites**  
1. **Network Configuration**  
   - **Wired Connection**: Controller IP is `192.168.5.1`. Set the computer to a fixed IP within the same subnet.  
   - **Wireless Connection**: Controller IP is `192.168.1.6`.  
   - Use the `ping` command to test the controller IP and ensure network connectivity.  

2. **System Requirements**  
   - **Operating System**: Ubuntu 22.04  
   - **ROS Version**: ROS2 Humble  

---

### **Installation and Configuration Steps**  

#### **1. Source Code Compilation**  
1. Download the source code:  
   ```bash
   mkdir -p ~/dobot_ws/src
   cd ~/dobot_ws/src
   git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4.git
   cd ~/dobot_ws
   ```

2. Compile the source code:  
   ```bash
   colcon build
   source install/local_setup.sh
   ```

3. Set environment variables:  
   ```bash
   echo "source ~/dobot_ws/install/local_setup.sh" >> ~/.bashrc
   ```

4. Configure the robotic arm connection IP (default for wired connection):  
   ```bash
   echo "export IP_address=192.168.5.1" >> ~/.bashrc
   ```

5. Specify the robotic arm model (select based on the actual model):  
   ```bash
   # Example: CR5 model
   echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
   ```
   - Supported models: CR3, CR5, CR7, CR10, CR12, CR16, CR20, E6 (ME6).  

6. Apply the configuration:  
   ```bash
   source ~/.bashrc
   ```
   - To modify the configuration, edit the `~/.bashrc` file using a text editor.  

---

### **Feature Demonstrations**  

#### **1. Simulation Environment Usage**  
- **RViz Model Loading**:  
  ```bash
  ros2 launch dobot_rviz dobot_rviz.launch.py
  ```
  - Used for visualizing the robotic arm model.  
![rviz](/image/rviz.jpg)
- **MoveIt Virtual Demo**:  
  ```bash
  ros2 launch dobot_moveit moveit_demo.launch.py
  ```
  - Drag the joint angles and click "Plan and Execute" to observe the motion.  
![moveit](/image/moveit.jpg)
- **Gazebo Simulation**:  
  ```bash
  ros2 launch dobot_gazebo dobot_gazebo.launch.py
  ```
  - Launches the Gazebo simulation environment.  
![gazebo](/image/gazebo.jpg)
- **Gazebo and MoveIt Integration**:  
  1. Launch Gazebo and MoveIt:  
     ```bash
     ros2 launch dobot_gazebo gazebo_moveit.launch.py
     ros2 launch dobot_moveit moveit_gazebo.launch.py
     ```
  2. Drag the robotic arm in MoveIt and execute "Plan and Execute"; the motion will synchronize with Gazebo.  
  ![service](/image/node.jpg)

---

#### **2. Controlling a Real Robotic Arm**  
1. **Connect to the Robotic Arm**:  
   ```bash
   ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
   ```

2. **View Service List**:  
   ```bash
   ros2 service list
   ```
  - ![service](/image/service.jpg)
3. **Control the Robotic Arm with MoveIt**:  
   ```bash
   ros2 launch dobot_moveit dobot_moveit.launch.py
   ```
   ![service](/image/dobot_moveit.jpg)
   - **Note**: Ensure the robotic arm is in remote TCP mode and enabled.  

4. **Enable the Robotic Arm**:  
   - Use `rqt service caller` to invoke the service:  
     ```
     /dobot_bringup_ros2/srv/EnableRobot
     ```
     ![gazebo](/image/rqt.jpg)

5. **Key Topics**:  
   - `/joint_states_robot`: Joint angle information.  
   - `/dobot_msgs_v4/msg/ToolVectorActual`: Cartesian position information.  
   ![gazebo](/image/topic.jpg)
---

### **Essential Toolkit Installation**  

#### **1. Gazebo Installation**  
- Installation command:  
  ```bash
  sudo apt install ros-humble-gazebo-*
  ```
- Environment variable configuration:  
  ```bash
  echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
  ```
- Test run:  
  ```bash
  ros2 launch gazebo_ros gazebo.launch.py
  ```

#### **2. MoveIt Installation**  
- Installation command:  
  ```bash
  sudo apt-get install ros-humble-moveit
  ```

---

### **Notes**  
- Ensure correct network configuration to avoid connection failures due to IP settings.  
- When operating a real robotic arm, strictly follow safety protocols to prevent unintended movements.  
- In the simulation environment, test motion planning before applying it to real-world scenarios.  

--- 

**Document Version**: V4  
**Last Updated**: April 2, 2025
