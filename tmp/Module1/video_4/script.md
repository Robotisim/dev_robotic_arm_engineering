# Notes

The collision element defines its shape the same way the visual element does. It can have it's own origin

You might be wondering what's the point if we're just repeating the visual tag. But we won't always be doing that. Visualising a mesh is a simple enough process but collision detection can get computationally heavy for complex meshes. And one can uses the accurate model for the visuals while resorting to a simpler mesh for collision detection.

## Inertia

We need to give our objects a physical presence to simulate them effectively. The two most important properties are mass and the inertia tensor which is analogous to mass for rotational motion.

## Robot State Publisher

We do have a way of definining where every part is , but how exactly is ros2 able to simulate movement, since we haven't directly given it anything to work with yet to do with motion.
There is a joint state publisher working in the background that sends a message indicating a movement in whatever joint is being moved.
This information goes to another program called Robot State Publisher, this keeps track of all of these messages along with access to the URDF.
Remember how we talked about using joint rotation to model the kinematics of the whole arm. This is where that magic happens.
Utilising information about joint movement along with information about all the links and joints present in the robot, it is able to generate the forward kinematics for the robot which then gets simulated in rviz.
To be more specific, it publishes the transforms of all the links and joints to a ros2 topic called tf2. Which is what rviz will use to visualise the robot.

## Urdf

- Start with robot name
- Name of first link. Usually base_link.
- define what it looks like, A cylinder of length 1 and radius 0.4.

## Urdf links

- add a second link.
- it will be a cuboid box with dimensions 0.5 0.1 and 0.2
- we will give it a rotation and position 
- If we simply run this we will get an error.
- For more than one link, there must be a joint that connects to the 2
- so we create a joint, there are multiple types of them.
    - for this demonstration we use a revolute. All that means is that it does rotational motion.
- For that to happen, we much specify the axis of rotation.
- And then some limits to its motion, As to how much it can move and how fast.
- Then we establish a parent joint as our base_link
- and a child link as our new link_1

## Urdf Collisions

- Visuals are like illusions. They have no matter and cannot be interacted with.
- To add mass, first we add a collision body. This gives it a physical presence that helps us identifiy when the robot collides with another object.
- Then we give it a value for mass and one for interia or moment of inertia.
- Mass: linear motion
- Inertia: rotational motion

## Xacro 

- Modifiyable version of urdf, that allows us to make use of variables, math , macros to streamline our urdf creation.
- simple example of a variable to give you an idea.


## Intro

We mentioned before that robots are a system of joints and links. Well how do we describe such a system. For  that we use a RDF file.

URDF stands for Unified Robot Description Format. It is an XML-based file format used primarily within the Robot Operating System (ROS) ecosystem to mathematically describe the physical and structural properties of a robot.Think of it as a digital blueprint that tells software exactly what a robot looks like, how it is built, and how it moves.
