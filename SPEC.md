# Robot Specifications

## Overview

The robot consistes of:

One Revolute Joint (R).
One Prismatic Joint (P).

The system has to support two ways of controlling and moving the arm:

1. Joint-space control: User specifies the desired position of individual joints to move the robot
2. Cartesian-space control: The user specifies the desired TCP (Tool Centre Point) coordinates, and the system calculates the required joint positions using inverse kinematics.

Implementation should be simple and effective.

## Robot Specifications

### Physical Parameters

Parameter                          Symbol           Value
Base height/TCP height             h1               0.15m
Base radius                        Rb               0.15m
Max prismatic extension            L2               0.40m
Number of joints                   DOF              2

### Joint specifications

Joint 1 - Revolute
- Joint type: Revolute
- Rotation axis: Vertical Z-axis
- Variable: θ1
- Minimum position: 0°
- Mximum position: 360°
- Internal representation: Radians

Joint 2 - Prismatic

- Joint type: Revolute
- Movement direction: Horizontal, along the local x-axis of the arm
- Variable: d2
- Minimum position: 0 m
- Mximum position: 0.40 m

The Prismatic displacement is measured from the revolute joint's rotation axis.

## Coordinate System

The robot uses a Cartesian coordinate system:

X: Horizontal direction.

Y: Horizontal direction perpendicular to X.

Z: Vertical direction.

The revolute joint rotates around the z-axis

The prismatic joint extends along the rotating arm's x-axis.

The TCP is located at the endpoint of the prismatic arm.

Its vertical coordinate remains fixed at 0.15 m.

## Functional Requirements

### 01: Simulation

The system must:

- Load the robotic arm into MuJoCo.

- Display the robot using the MuJoCo viewer.

- Simulate revolute and prismatic joint movement.

- Display the robot's current configuration.

### 02: Joint-Space Control

The system must accept commands specifying individual joint positions.

Example:

Joint 1 = 10 degrees
Joint 2 = 0.30 meters

Expected result:

- Rotate Joint 1 to 10 degrees.

- Extend Joint 2 to 0.30 meters.

The system must validate joint limits before executing commands.

### 03: Forward Kinematics

The system must calculate the TCP position from the current joint positions.

Equations:

x = d2 × cos(θ1)

y = d2 × sin(θ1)

z = h1

### 04: Inverse Kinematics

The system must calculate the required joint positions from a given TCP position.

Equations:

θ1 = atan2(y, x)

d2 = sqrt(x² + y²)

z = h1

The system must reject unreachable target positions.

### 05: Cartesian-Space Control

The system must accept target TCP coordinates.

Example:

X = 0.10 m
Y = 0.30 m
Z = 0.15 m

Expected result:

- Calculate the required joint angle.

- Calculate the required linear extension.

- Validate the resulting joint positions.

- Send the joint targets to the controller.

- Move the robot towards the requested position.

### 06: Text-Based Commands

The system should support structured text commands for:

- Setting individual joint positions.

- Setting multiple joint positions.

- Moving the TCP to specified Cartesian coordinates.

The initial implementation will use a simple command parser rather than an LLM.

### 07: Position Feedback

The system must provide:

- Current revolute joint position.

- Current prismatic joint position.

- Current TCP coordinates.

- Target joint positions.

## Kinematic Constraints

The robot has two degrees of freedom. It's reachable TCP positions satisy:

sqrt(x² + y²) <= 0.40 m

z = 0.15 m

The target must also satisy joint limits.

The robot cannot independently control it's TCP height or orientation.

When the requested radial distance is zero, the rotation angle is undefined. The implementation should preserve the current revolute joint angle in this special case.

## Future Extensibility

Future Extensibility

The architecture should allow future support for robotic arms with additional joints.

Potential extensions include:

- 4-DOF robotic arms.

- 6-DOF robotic manipulators.

- 8-DOF robotic manipulators.

- Jacobian-based inverse kinematics.

- Damped Least Squares (DLS).

- More advanced motion planning.

- Natural-language command interpretation.

The initial implementation will use analytical inverse kinematics for the 2-DOF robot.