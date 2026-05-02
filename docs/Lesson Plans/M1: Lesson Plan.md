# Module 1: ROS 2 Robotic Arm Getting Started — URDF, Xacro, and RViz

## Module purpose

This module should introduce students to **how a robotic arm is represented inside ROS 2**. The goal is not yet to teach motion planning, inverse kinematics, or task-space control in detail. The goal is to help students understand that before a robotic arm can move intelligently, ROS 2 first needs a clear description of the robot’s physical structure.

In this module, students will create and visualize a robotic arm using **URDF**, then improve the structure using **Xacro**, and finally inspect the arm in **RViz** using joint states and robot state publisher.

The deeper concepts like **task space, end-effector pose, forward kinematics, inverse kinematics, and motion planning** should be introduced only lightly here as future motivation. They should become the focus of the next modules.

---

# Learning outcomes

By the end of this module, students should be able to:

* Understand why robotic arms need a robot description before control or planning.
* Explain what URDF is and how it describes links, joints, visuals, collisions, and inertial properties.
* Create a basic shape-based robotic arm using URDF.
* Understand the relationship between **links**, **joints**, and the robot’s kinematic chain.
* Launch the robot description in ROS 2.
* Visualize the robotic arm in RViz.
* Use joint states to move or inspect the robot’s joints.
* Understand why Xacro is used to make larger robot descriptions cleaner and reusable.
* Explore a Panda-style robotic arm description and understand how real robot URDF/Xacro files are structured.
* Understand at a basic level that joint movement eventually connects to end-effector movement.

---

# What should be taught now vs later

## Teach in Module 1

These should be the main focus:

* URDF structure
* Links and joints
* Parent-child relationships
* Joint types: fixed, revolute, continuous, prismatic
* Joint origin and axis
* Visual geometry
* RViz visualization
* Robot State Publisher
* Joint State Publisher / GUI
* Xacro basics
* Reading a real robotic arm description, such as Panda

## Mention lightly, but do not go deep yet

These concepts should be introduced only as motivation:

* Joint space
* Task space
* End-effector
* Forward kinematics
* Inverse kinematics
* Motion planning
* MoveIt

For example, you can say:

> “Right now, we are only describing the robot. Later, when we want the gripper to reach an object, we will need concepts like task space, forward kinematics, inverse kinematics, and motion planning. But all of those depend on having a correct robot model first.”

This keeps students excited without overloading them.

---

# Storytelling flow for Module 1

## Main story

Before we can command a robotic arm to pick, place, weld, paint, or manipulate objects, ROS 2 needs to understand what the robot looks like, how its joints are connected, and how each link moves relative to the others.

So the first question is not:

> “How do we move the robotic arm?”

The first question is:

> “How do we describe the robotic arm so ROS 2 understands its body?”

That is the foundation of this module.

---

# Question-based lesson sequence

## 1. What are we trying to do?

We are trying to bring a robotic arm into ROS 2 so we can visualize it, inspect its joints, and prepare it for future control and motion planning.

Before controlling the arm, we need to define its structure: base, links, joints, axes, and how everything is connected.

---

## 2. What problem appears?

A real robotic arm has many physical parts, but ROS 2 does not automatically know those parts.

If we do not describe the robot correctly, RViz, TF, motion planning, and control systems will not understand how the arm is built or how it should move.

---

## 3. What do we need to solve it?

We need a robot description file that defines:

* The robot’s links
* The robot’s joints
* Parent-child relationships
* Joint types
* Joint axes
* Visual shapes
* Collision shapes
* Origins and transforms

This description becomes the base model that other ROS 2 tools can use.

---

## 4. What is this called in ROS 2 robotics?

This is called a **URDF**, which means **Unified Robot Description Format**.

URDF is the standard way to describe a robot’s physical and kinematic structure in ROS.

---

## 5. How does it work?

URDF describes the robot as a tree of links and joints.

Links represent physical parts of the robot, while joints describe how one link is connected to another and how it can move.

Example:

```text
base_link → shoulder_joint → upper_arm_link → elbow_joint → forearm_link → wrist_joint → gripper_link
```

This structure allows ROS 2 to understand how motion travels from the base of the robot to the end-effector.

---

## 6. How do we implement it?

We first create a simple shape-based robotic arm using basic URDF geometry such as boxes, cylinders, and spheres.

Then we launch it using:

* `robot_state_publisher`
* `joint_state_publisher`
* `joint_state_publisher_gui`
* RViz

After that, we introduce **Xacro** to make the robot description cleaner and reusable.

---

## 7. How do we know it worked?

We know it worked when:

* The robot appears correctly in RViz.
* Links are connected in the correct order.
* Joints rotate around the expected axes.
* The TF tree looks correct.
* Changing joint values updates the robot pose visually.
* The robot model does not appear broken, disconnected, or incorrectly oriented.

---

# Suggested module lessons

## Lesson 1: Why robot description matters

### Goal

Explain why robotic arms need a model before control, planning, or simulation.

### Key points

* ROS 2 needs to know the robot’s body structure.
* A robotic arm is not just one object; it is a chain of links and joints.
* The end-effector movement depends on all previous joints.
* URDF is the foundation for visualization, TF, control, and planning.

### Learning outcome

Students understand why robot description is the first step before moving toward control or motion planning.

---

## Lesson 2: Understanding links and joints

### Goal

Teach the basic building blocks of a robotic arm model.

### Key points

* Link = physical body part
* Joint = connection between two links
* Parent link and child link
* Fixed joints vs movable joints
* Revolute, continuous, prismatic joints
* Joint axis
* Joint origin

### Storytelling question

> “If a robotic arm is made of multiple parts, how does ROS 2 know which part is connected to which?”

### Learning outcome

Students can explain how links and joints form a robotic arm chain.

---

## Lesson 3: Creating a simple shape-based URDF robotic arm

### Goal

Build a basic robotic arm model using simple shapes.

### Key points

* Create package structure
* Add URDF folder
* Define `base_link`
* Add shoulder link
* Add upper arm link
* Add elbow joint
* Add forearm link
* Add wrist/gripper link
* Use basic geometry

### Suggested project

Create a 3-link robotic arm using simple cylinders and boxes.

### Learning outcome

Students can create a simple URDF robotic arm from scratch.

---

## Lesson 4: Visualizing the robotic arm in RViz

### Goal

Launch and inspect the robotic arm in RViz.

### Key points

* `robot_state_publisher`
* `joint_state_publisher`
* `joint_state_publisher_gui`
* RViz robot model display
* Fixed frame
* TF visualization

### Storytelling question

> “Now that we wrote the robot description, how do we check whether ROS 2 understands it correctly?”

### Learning outcome

Students can launch and visualize their robotic arm in RViz.

---

## Lesson 5: Understanding joint movement and joint angles

### Goal

Show how changing joint values affects the robot’s shape in RViz.

### Key points

* Joint angle
* Joint limits
* Joint axis
* Rotation around axis
* Relationship between joint movement and link movement
* Basic idea of joint space

### Important note

This is where you can lightly introduce **joint space**.

You can say:

> “When we move the robot by changing joint values, we are working in joint space. Later, when we want the gripper to reach a point in 3D space, we will move toward task space.”

### Learning outcome

Students understand that robotic arms can be described and inspected through joint values.

---

## Lesson 6: Why URDF becomes difficult for bigger robots

### Goal

Show why plain URDF becomes hard to maintain.

### Key points

* Repeated XML structure
* Long robot descriptions
* Hard-coded values
* Difficult updates
* Need for reusable components

### Storytelling question

> “If our simple robot already has many repeated parts, what happens when we describe a real 7-DOF robotic arm?”

### Learning outcome

Students understand why Xacro is useful.

---

## Lesson 7: Introduction to Xacro

### Goal

Convert or compare the simple URDF structure with Xacro.

### Key points

* Xacro makes robot descriptions reusable.
* Variables can store dimensions.
* Macros can define repeated link/joint patterns.
* Xacro generates URDF.

### Learning outcome

Students understand how Xacro improves maintainability.

---

## Lesson 8: Exploring Panda robotic arm description

### Goal

Show students a more realistic robotic arm description.

### Key points

* Panda has multiple links and joints.
* Real robot descriptions are split into multiple Xacro files.
* Meshes are used instead of only basic shapes.
* Joint limits and kinematic structure are more detailed.
* The model is prepared for future MoveIt and planning use.

### Learning outcome

Students can inspect a real robotic arm description and understand its structure without needing to master everything yet.

---

# Recommended module structure

## Module 1 title

**Getting Started with Robotic Arms in ROS 2: URDF, Xacro, and RViz**

## Module 1 project

**Build and visualize a simple robotic arm in ROS 2, then inspect a real Panda-style Xacro robot description.**

## Module 1 final outcome

By the end of the module, the student should have:

* A ROS 2 package for robotic arm description
* A simple shape-based URDF arm
* A launch file to visualize it in RViz
* A working joint state GUI
* A basic understanding of links, joints, joint axes, and joint angles
* A basic understanding of Xacro
* A first look at a real robotic arm model like Panda

---

# What should be saved for Module 2

Module 2 can focus on:

* TF tree in more detail
* Forward kinematics
* Joint space vs task space
* End-effector pose
* How joint angles affect gripper position
* How the robot’s description becomes useful for kinematics

A good Module 2 title could be:

**From Joint Angles to End-Effector Pose: Understanding Robotic Arm Kinematics in ROS 2**

That module can answer:

* If joints move, where does the gripper go?
* What is forward kinematics?
* What is task space?
* What is joint space?
* How does ROS 2 track transforms?
* How does TF represent the robotic arm chain?

---

# Small checklist for Youssef’s delivery

For each lesson in this module, Youssef should make sure:

* Start with a question or problem.
* Explain why the concept matters.
* Connect the concept to robotic arms.
* Show the implementation.
* Test the result in RViz.
* Summarize what was learned.
* Connect the lesson to the next step.

Example delivery style:

> “We are not creating URDF just for visualization. We are creating the foundation that later allows the robot to understand its joints, transforms, end-effector position, and motion planning.”

That sentence connects Module 1 with the larger robotic arm learning path.

---

# Final recommendation

Do **not** go deep into task space in Module 1. Mention it only as a future concept.

Module 1 should focus on:

```text
Robot structure → URDF → links/joints → RViz → joint movement → Xacro → real robot model
```

Module 2 should focus on:

```text
Joint movement → TF → forward kinematics → end-effector pose → joint space vs task space
```

This keeps Module 1 clean, practical, and beginner-friendly while still preparing students for the technical robotic arm concepts coming next.
