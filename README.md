# Joystick teleoperation for Mobile Robots using ROS, Python
+ Simple Teleoperation using [PS4 from SONY](https://asia.playstation.com/ko-kr/accessories/dualshock4/)
<p align="left">
<img src="https://github.com/engcang/image-files/blob/master/joyteleop/ps4.png" width="300" hspace="50"/>
</p>

## Index
+ [Joystick Set up](#-joystick-set-up-and-install)
+ [Code explanation](#-code-explanation)
+ [Using code as ROS node](#-using-code-directly-or-as-ros-node)
+ [Robot Teleoperation Result](#-result-data-for-turtlebot2-and-turtlebot3)

<br>

## ● Joystick Set up and install
+ First install [**Joy** package](http://wiki.ros.org/joy) by
  ~~~shell
  $ sudo apt-get install ros-<your distro version>-joy
  ~~~
<br>

+ Do bluetooth pairing
<p align="left">
<img src="https://github.com/engcang/image-files/blob/master/joyteleop/pairing.JPG" width="400" hspace="80"/>
</p>
<br>

+ After that, give permission to access bluetooth controller
  ~~~shell
  $ roscore
  $ rosrun joy joy_node
  $ sudo chmod a+rw /dev/input/js0
  ~~~
  </br>

+ and check the data comes in using **rostopic echo**, while moving stick and pushing buttons of PS4
  ~~~shell
  $ rostopic list
  $ rostopic echo /joy
  ~~~
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/joyteleop/joy_echo.png" width="500" hspace="0"/>
</p>
<br>

+ data.buttons[0] is _rectangle_ button, [1] is _x_ button, [2] is _circle_ button and [3] is _triangle_ button each, the picture shows when I pushed _circle_ button <br>
+ data.axes[0] and data.axes[1] are Left stick's data corresponding to angular and linear _velocities_ each

<br><br><br>

## ● Code explanation
+ Import libraries and init class
  ~~~python
  import numpy as np
  import rospy
  import roslib
  import subprocess
  import time
  from geometry_msgs.msg  import Twist
  from sensor_msgs.msg import Joy
  import sys
  import signal

  def signal_handler(signal, frame): # ctrl + c -> exit program
          print('You pressed Ctrl+C!')
          sys.exit(0)
  signal.signal(signal.SIGINT, signal_handler)
  ''' class '''
  class robot():
      def __init__(self):
          rospy.init_node('robot_controller', anonymous=True)
          self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
          self.pose_subscriber2 = rospy.Subscriber('/joy',Joy,self.callback)
          self.rate = rospy.Rate(20)
  ~~~
  1.for turtlebot2, used **/mobile_base/commands/velocity** as velocity publishing topic, for turtlebot 3, should use **/cmd_vel** <br>
  Both codes were uploaded on this repository. <br>
  2.To get joystick's data, make subscriber under **'/joy'** topic

<br>

+ Class methods
  ~~~python
      def callback(self, data):
        global inn
        inn=0
        self.joy = data.buttons
        self.joy2= data.axes
        if np.shape(self.joy)[0]>0:
            inn=1
            self.nemo=self.joy[0]
            self.semo=self.joy[3]
            self.one=self.joy[2]
            self.x=self.joy[1]
        if np.shape(self.joy2)[0]>0:
            inn=1
            self.linear=self.joy2[1]
            self.angular=self.joy2[0]
        if inn==1:
            if self.joy[0]==0 and self.joy[1]==0 and self.joy[2]==0 and self.joy[3]==0 and self.joy2[0]==0 and self.joy2[1]==0:
                inn=0
            else:
                pass
    def moving(self,vel_msg):
        self.velocity_publisher.publish(vel_msg)

  ~~~
  3._**Callback**_ function is automatically implemented whenever data comes in under **joy** topic <br>
  4.When joy data is not empty, **globall inn** is replaced with **'1'** <br>

<br>

+ main code
  ~~~python
  ''' main '''
  if __name__ == '__main__':
   while 1:
       if inn==1:
          if turtle.nemo==1:
               vel_msg.linear.x=turtle.linear*0.4
               vel_msg.angular.z=turtle.angular*1.2
          elif turtle.semo==1:
               #subprocess.call('',shell=True)
               p=subprocess.Popen('rostopic pub /mobile_base/commands/reset_odometry std_msgs/Empty "{}"',shell=True)
               time.sleep(2)
               p.terminate()
          elif turtle.one==1:
               vel_msg.linear.x=turtle.linear*0.7
               vel_msg.angular.z=turtle.angular*2
          elif turtle.x==1:
               vel_msg.linear.x=0
               vel_msg.angular.z=0
          turtle.moving(vel_msg)        
       else:
           print('no data in')
       turtle.rate.sleep()
  ~~~
  5.when **global inn** is '1', do any tasks using joy buttons data and joy axes data <br>
  6.when **rectangle** button is pushed, move slowly and **circle** button is pushed, move fast <br>
  **X** button is pushed, stop and **triangle** button is pushed, reset _**Odometry**_ <br>

<br>

## ● Using code directly or as ROS node
+ git clone the codes first
  ~~~shell
  $ git clone https://github.com/engcang/PS4_Joystick_teleop_Mobile_Robots_ROS_Python.git
  ~~~
<br>

+ Run the code directly with ROS Master
  ~~~shell
  $ roslaunch turtlebot_bringup minimal.launch
  $ rosrun joy joy_node
  $ python PS4_Joystick_teleop_Mobile_Robots_ROS_Python/Joyteleop_turtlebot2.py
  ~~~
  1.**Have to run robot_bringup first!** <br>
  2.**Have to run joy_node** to get joystick data <br>
  3.run python code
  
<br>

+ Run the code after make it as Node
  ~~~shell
  $ cd ~/catkin_ws/src
  $ catkin_create_pkg <name> rospy roslib sensor_msgs geometry_msgs
  $ cd ~/catkin_ws && catkin_make
  $ cd ~/catkin_ws/src/<name> && mkdir scripts
  $ mv ~/PS4_Joystick_teleop_Mobile_Robots_ROS_Python/Joyteleop_turtlebot2.py ~/catkin_ws/src/<name>/scripts
  $ chmod +x Joyteleop_turtlebot2.py
  $ roscore
  $ rosrun <name> Joyteleop_turtlebot2.py
  ~~~
<br>

+ Run the code by ROS launch
  ~~~xml
  <node pkg="<name>" type="Joyteleop_turtlebot2.py" name="Joyteleop_turtlebot2" />
  ~~~
  Simply add this line into launch file you want to launch together

<br>

## ● Result data for turtlebot2 and turtlebot3
<p align="center">
<img src="" width="400" height="500" hspace="0"/>
</p>
