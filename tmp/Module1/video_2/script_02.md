# Video 2

Within each robotics projects theres often a lot going on. Let's try and get an overview of what exactly we are dealing with.

In this video we will explore 3 packages briefly.    

- robotic_arm_description
- robotic_arm_sim
- robotic_arm_bringup

- robotic_arm_control
- robotic_arm_hardware

## robotic_arm_description

This folder contains the robot models that we will use in the course.  Starting with the basic stick arm and the more complicated Panda arm. The models describe visual features of each arm but also the physical functions and limitations of each part of the arm, for example how much each joint can rotate. (Maybe add a snippet of code)

## robotic_arm_sim

This package will contain the assets for simulating environments in Gazebo. Things like the the ground plane, table etc. This package will also help gazebo visualise the panda arm that we will be using later.

## robotic_arm_bringup

Each package mentioned before had a specific purpose. bringup is the the bridge which connects all. This is our entry point, where we wire together information from all other packages as needed to run a completely functional.

## Example

Car

## description
- how it looks
- how many wheels it has 
- how each part looks

## sim

- assets of the roads. 
- pedestrians
- light source
- maybe some buildings depending on use case

## bringup

- One launch file for just a road and one car
- One launch file with two cars.
- customising as neccesary

