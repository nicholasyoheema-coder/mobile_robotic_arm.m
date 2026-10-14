# Mobile Manipulator Robot Simulation with Obstacle Avoidance

A MATLAB-based robotics simulation of a mobile manipulator robot that autonomously navigates around obstacles, approaches objects, and performs five pick-and-place tasks using grid-based path planning and arm inverse kinematics.

## Project Overview

This project demonstrates how a mobile robot equipped with a robotic arm can navigate an environment, avoid obstacles, and transport objects from their initial positions to designated goal positions.

The robot is modelled and animated in MATLAB using the Robotics System Toolbox. Its navigation uses a grid-based path planner, while the arm performs scripted pick-and-place movements.

## Features

- **Custom Mobile Robot:** A four-wheel mobile platform with a three-joint robotic arm and gripper.
- **Obstacle Avoidance:** Plans collision-aware paths around obstacles using an inflated robot footprint and safety margin.
- **Dijkstra Path Planning:** Searches a two-dimensional grid to find a feasible path to a suitable position near each target.
- **Path Optimization:** Shortens the planned route using line-of-sight checks.
- **Inverse Kinematics:** Calculates arm joint configurations for target positions using an analytical inverse-kinematics function.
- **Five-Block Pick and Place:** Moves five blocks from their starting positions to designated goal locations.
- **Robot Animation:** Visualizes base movement, wheel rotation, arm motion, block transport, and the robot's trajectory.
- **Clearance Tracking:** Tracks the minimum calculated robot-to-obstacle clearance during base movement.
- **3D Visualization:** Displays the robot, obstacles, blocks, destination markers, planned paths, and movement trail.

## Technologies Used

- MATLAB
- Robotics System Toolbox
- `rigidBodyTree` robot model
- Homogeneous transformations and coordinate frames
- Dijkstra's algorithm
- Analytical inverse kinematics
- MATLAB graphics and animation

## How It Works

1. **Environment Setup:** Defines the workspace boundaries, obstacles, five starting block positions, and five destination positions.
2. **Robot Modelling:** Builds the mobile base, revolute arm joints, wheels, gripper, and tool frame.
3. **Obstacle Inflation:** Expands obstacles and accounts for the robot footprint and safety margin during navigation planning.
4. **Path Planning:** Uses Dijkstra's algorithm on a two-dimensional grid to find a collision-aware route to a suitable standing position near a target.
5. **Path Shortening:** Removes unnecessary intermediate waypoints when a direct, collision-free segment is available.
6. **Navigation:** Rotates and drives the mobile base along the planned waypoints while keeping the arm folded.
7. **Pick and Place:** Uses inverse kinematics to calculate arm configurations, then animates the approach, lowering, lifting, carrying, and placement sequence.
8. **Repeat and Evaluate:** Repeats the process for all five blocks and reports the calculated minimum obstacle clearance.

## Requirements

- MATLAB
- Robotics System Toolbox
- A desktop MATLAB environment capable of displaying 3D figures

## How to Run

1. Clone or download this repository.
2. Open MATLAB.
3. Navigate to the folder containing the project file.
4. Open `obstacleavoidance_mobilerobot.m` (or the actual filename used in your repository).
5. Run the script.

The simulation creates a 3D environment and begins processing the five pick-and-place tasks. Progress messages appear in the MATLAB Command Window.

## Configuration

Important parameters can be modified near the beginning of the script:

| Parameter | Description |
|---|---|
| `W.bx`, `W.by` | Allowed robot-centre workspace boundaries |
| `W.res` | Grid resolution for path planning |
| `W.rRobot` | Approximate robot footprint radius |
| `W.safe` | Additional safety margin |
| `W.obs` | Obstacle positions and radii |
| `blockStart` | Initial positions of the five blocks |
| `blockGoal` | Destination positions for the blocks |
| `robotStart` | Initial mobile base position |
| `standoff` | Desired base-to-target distance |
| `liftH` | Target height offset used for arm approach |
| `S.dt` | Animation pause duration |

## Expected Result

The simulation visualizes the mobile manipulator navigating around obstacles, approaching each block, moving the arm through its pick-and-place sequence, and transporting all five blocks to their designated destinations.

At the end, the simulation displays a completion message and reports the calculated minimum robot-to-obstacle clearance.

## Limitations

- This is a MATLAB kinematic and visualization simulation, not a complete physics-based robot simulator.
- The gripper and object handling are scripted; real contact forces, gripping forces, and physical object dynamics are not simulated.
- Obstacle avoidance is based on the implemented geometric model and planning assumptions; it does not guarantee collision-free motion for every part of the arm.
- The clearance value is based on the implemented base-motion calculation and is not a complete three-dimensional collision validation.
- No physical robot hardware is controlled by this script.

## Future Improvements

- Integrate a physics-based simulator such as Gazebo.
- Add a more complete arm and gripper collision model.
- Implement sensor-based obstacle detection and dynamic replanning.
- Improve trajectory generation and joint-space motion control.
- Connect the system to ROS 2 and real robot hardware.

## Author

**Yoheema Nicholas**

Robotics and Automation Engineering

## License

Add a license to this repository if you intend to distribute or reuse the project publicly.
