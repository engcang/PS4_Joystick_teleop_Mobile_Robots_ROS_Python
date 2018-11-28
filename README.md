# Joystick teleoperation for Mobile Robots using ROS, Python
+ Simple Teleoperation using [PS4 from SONY](https://asia.playstation.com/ko-kr/accessories/dualshock4/)
<p align="left">
<img src="https://github.com/engcang/image-files/blob/master/joyteleop/ps4.png" width="300" hspace="50"/>
</p>

## Index
+ [Joystick Set up](#)
+ [Code explanation](#)
+ [Using code as ROS node](#)
+ [Robot Teleoperation Result](#)

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

+ 

<br>

+ 

<p align="center">
<img src="" width="500" hspace="0"/>
</p>

</br></br>

## ● Code explanation
+ Import libraries and 
  ~~~python
  ~~~
  1.
  2.

<br>

+ 
  ~~~python
  ~~~
  3.

<br>

## ● 
+ 
  ~~~shell
  ~~~
<br>

## ● 
+ 
  ~~~shell
  ~~~
<br>

+ Run the code directly with ROS Master
  ~~~shell
  ~~~
<br>

+ Run the code after make it as Node
  ~~~shell
  $ cd ~/catkin_ws/src
  $ catkin_create_pkg <name> rospy roslib std_msgs
  $ cd ~/catkin_ws && catkin_make
  $ cd ~/catkin_ws/src/<name> && mkdir scripts
  $ mv ~/HC-SR04-UltraSonicSensor-ROS-RaspberryPi/ROS_sonar_sensor.py ~/catkin_ws/src/<name>/scripts
  $ chmod +x ROS_sonar_sensor.py
  $ roscore
  $ rosrun <name> ROS_sonar_sensor.py
  ~~~
<br>

+ Run the code by ROS launch
  ~~~xml
  <node pkg="<name>" type="ROS_sonar_sensor.py" name="ROS_sonar_sensor" />
  ~~~
  Simply add this line into launch file you want to launch together

<br>

+ Result data by _**rostopic echo /sonar_dist**_
<p align="center">
<img src="https://github.com/engcang/image-files/blob/master/sonar_sensor/ROS_topic.gif" width="400" height="500" hspace="0"/>
</p>
