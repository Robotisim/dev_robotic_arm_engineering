# M1_V03_Frames_Links_Joints_and_TF_Mental_Model_[A]

## Intro

So what exactly is a robotic arm? Well..
A robotic arm is a reprogrammable mechanical arm which functions similar to a human arm.

So any movement you can do with you arm, the robotic arm can mostly emulate. Try moving your arm about, you will notice there are only rotations happening at certain places causing all the motion. These are your joints and so their counterpart in robotics are called the same. Then we have connections between each joint, these we will call links. They do not possess the ability to move themselves but rather move as a consequence of joint rotation. Each robotic arm will have a base and an end. Just like we have hands at the end of our arms, we will have a gripper similary in the robotic arm. These are more generally called end effectors, and can appear in all sorts of forms. 

So to summarise so far, we have a robotic arm that moves uses joints, where each joint is connected by a link. And together these links and joints along with the end effector create a whole arm.

As you can see all we have access to for now is information about how much each joint is rotated. Now how can we go from that to figuring out where exactly each part is. 

## Concept

Keeping track of each individually can become quite messy. To make keep our workflow clean we introduce the frames. So what is a frame. A frame or coordinate frame is a frame of reference. We typically create it as such that Forward facing is the X direction, left is y and upwards is Z. The process goes as follows:

- Set our base frame at the base link
- Move to the next link, set up another frame depending on its position and orientation
- Keep going till the end.

Something to note is that movement of any joint only affects the position of links infront of it, and not behind it. This naturally forms a parent child relationship that we can utilise to further organise the structure.

Base will be the parent. Then link1 will be it's child.
Link 2 will be the child of link1. And so forth
Note that a parent can have multiple children but a child cannnot have more than one parent. This would cause issues since we use the location of the parent to find the location of the child.

With all of this in place we can now effectively figure out the position of each part through simple matrix multiplications. These can also be called transforms.

We mentioned before that all we have access to is how much each joint rotates. Using this system, we can then use the information to generate a rotational transform for the new orientation of the frame related to that joint and then multiply all the rest of the joints that are children further down the tree with the same transform to find out there position and orientation.

## Outro

Mention TF trees.
