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
