# Shared ROS 2 interfaces

- `Object.msg`: stable scene ID for the supported cube, colour, pose, dimensions and confidence.
- `Objects.msg`: frame/timestamp, revision and a collection of objects.
- `MoveXYZ.action`: framed XYZ, tolerance and explicit execution flag; returns joints, residual and outcome.
- `Task.action`: operation, pose/joints/target, destination, object geometry and explicit execution flag; returns success/detail/duration and stage feedback.

Coordinates are metres; robot angles are radians; Panda finger position is metres. Real-state provenance is command-derived, simulation-state provenance is Gazebo. Clients never infer real motor arrival from an action's software success field.
