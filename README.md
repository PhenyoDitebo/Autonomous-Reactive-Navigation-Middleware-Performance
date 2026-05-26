# Autonomous Reactive 'Naive' Navigation Algorithm
A first-principles approach to robotics middleware, implementing autonomous obstacle avoidance and real-time telemetry using Bash scripting to interface directly with the ROS 2 CLI.

## Project Overview
While most ROS 2 applications utilize high-level client libraries (C++/Python), this project explores the underlying communication layer. By using the shell to manage parameters and service calls, I characterized the relationship between script latency and physical hardware constraints.

<img width="1100" height="368" alt="image" src="https://github.com/user-attachments/assets/8265f800-494c-40bc-acec-d1e1993be773" />
<table border="0">
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/1852e62c-834f-48af-a1bb-7e5ddc223342">
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/9a562ff3-cf34-4390-91ef-7d4f4e4e87ea">
    </td>
  </tr>
</table>


### Key Technical Achievements:
* **Deterministic Control Loop:** Implemented a 10Hz heartbeat and managed timing offsets to ensure stable command execution despite the overhead of shell-based service calls.
* **Mechanical Empathy & Safety Logic:** Applied physical constraints (e.g., $0.3\pi$ rad/s angular velocity limits) to prevent LiDAR "motion blur" and ensure reliable obstacle detection.
* **Double Redundant Shutdown:** Engineered safety fallbacks that publish a final _geometry_msgs/msg/Twist_ with zero-velocity vectors to clear the command buffer and prevent ghost drifting if previous logic had not caught the break in logic.

#### Link to Presentation Material: [https://drive.google.com/file/d/1At5ZZYHPsSMsBmi-iuNjZXaGwxg54GBg/view?usp=sharing](https://www.youtube.com/live/MhbMvyM5W74?si=jVJa68HdnzgqKa08)
## Features

### 1. Naive Obstacle Avoider (obstacle_avoider.bash)
* **Predictive Braking:** Calculates _time_to_move_ variable based on distance-to-threshold to avoid high-momentum collisions.
* **Directional Heuristics:** Compares left vs. right ray ranges to dynamically select the path with maximum clearance.
* **Fail-Safe Logic:** Automatically assigns a large value (10.0m) to "inf" or null sensor returns to prevent logic crashes during computer/sensor failure.

### 2. Robot Statistics & Telemetry (robot_statistics.bash)
* **State Tracking:** Tracks absolute distance covered, 3D position ($x, y, z$), and orientation (Roll, Pitch, Yaw).

## Technical Implementation Details

### Velocity & Perception Balance
I capped the angular velocity at 0.942 rad/s ($0.3\pi$). This specific value was chosen because:
* Lower values (0.1 rad/s): Too slow for efficient navigation.
* Higher values: Caused the LiDAR scan (SLAM) to skip or blur, leading to collisions.

## Software Tools Used
* ROS/ROS2
* Python
* TurtleBot3 Simulation and/or Physical TurtleBot3 Robot

## Learning Outcomes
* Middleware Performance: Gained intuition on how ROS2 and CLI-based publishing affect real-time system performance.
* Logic Abstraction: Successfully bypassed high-level APIs to understand how race conditions manifest at the OS level.
* System Integration: Bridged the gap between raw sensor data (LiDAR/Odom) and physical actuator response.
