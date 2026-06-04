# script

## intro

- We went from a theoretical concept of links and joints to their representation in a URDF. Now we will look at an actual example that we'll be using in this course.

### xacro

  <xacro:arg name="name" default="stick_arm"/>
  <xacro:arg name="prefix" default=""/>

defaults
- name: stick_arm 
- prefix: empty

- each variable simply replaces wherever it is repeated with the value that is set before.

- When utilising this Xacro file in a launchfile we can pass values for name and prefix which will overwrite the defaults.
