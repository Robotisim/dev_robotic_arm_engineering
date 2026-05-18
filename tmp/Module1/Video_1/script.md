# Script

## Introduction to the module

Robotics can seem like a complicated subject to decipher but it can be broken down into some easy to understand parts. The first of which, and perhaps most fundamental to our journey will be the robot description. It will get referenced again and again in later modules as well. 

Put simply, it is a way of letting ROS2 know what kind of robot it is dealing with. And if the model we present of our robot is inaccurate, our code will fail to produce any useful output. Models are everything when it comes to computation

### Example

Take your own arm. It can be broadly broken down to the shoulder, elbow and wrist. The joints can turn a certain amount and the length of each part is variable too. 


## Goal

By the end of the module, you will be well versed in understand how exactly ROS2 and Gazebo identify a robot. How the individual parts link together to create a whole entity. Once you have a foundational understanding you will be able to visualise the robotic arm in Gazebo and RVIZ along with some basic motion to begin to form an understanding of the workflow that will be used much more later on in the modules.

Since alot of processes will need to know information about the Robot, it would help to have a way to make the it be accessible to everything whenever needed. We resolve this by having a dedicated Robot Description that contains all the information about the robot. 

### example

Take a car for example. It has a certain shape that we can roughly model as a cuboid along with 4 wheels, two of which can turn and 2 of which can be driven forward or backwards. These functionalities and limitations of each part are what will help determine how the algorithms we use will affect the motion of the vehicle.

The algorithms by themselves do not know anything about these. It is our job to inform them of what parts are present and in what ways they can be used. Travelling in a straight line can be done at various speeds. But what the limit is will be determined by us. The algorithms can then later on determine how to optimise within those limits.
