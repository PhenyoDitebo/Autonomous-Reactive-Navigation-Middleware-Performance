# Autonomous Reactive Navigation Algorithm
A first-principles approach to robotics middleware, implementing autonomous obstacle avoidance and real-time telemetry using Bash scripting to interface directly with the ROS 2 CLI.

## 📌 Project Overview
While most ROS 2 applications utilize high-level client libraries (C++/Python), this project explores the underlying communication layer. By using the shell to manage parameters and service calls, I characterized the relationship between script latency and physical hardware constraints.

### Key Technical Achievements:
* **Deterministic Control Loop:** Implemented a 10Hz heartbeat and managed timing offsets to ensure stable command execution despite the overhead of shell-based service calls.
* **Mechanical Empathy & Safety Logic:** Applied physical constraints (e.g., $0.3\pi$ rad/s angular velocity limits) to prevent LiDAR "motion blur" and ensure reliable obstacle detection.
* **Double Redundant Shutdown:** Engineered safety fallbacks that publish a final _geometry_msgs/msg/Twist_ with zero-velocity vectors to clear the command buffer and prevent ghost drifting if previous logic had not caught the break in logic.

## 🛠 Features
