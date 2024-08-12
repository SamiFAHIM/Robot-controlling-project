# Robot-Control-Card-Software C++
## Overview
Welcome to the source code repository for the Motor Control Card of our autonomous robot, designed and developed by the Clubelek Association, representing our university INSA Lyon at the Eurobot Contest. This control card is a critical component of our robot's system, handling motor regulation, odometry, and localization. The software running on this card ensures precise control of the robot's movement and accurate tracking of its position, which is essential for autonomous navigation.
 
The code is designed to be seamlessly interfaced within a ROS (Robot Operating System) project, allowing the control card to interact with higher-level nodes in the robotic architecture. Multiple ROS nodes are implemented within the software to publish odometry, velocity dataand more, making it compatible with complex robotic systems that require real-time data exchange.

## Features
- Motor Regulation: Smooth and efficient control of motor speed and direction to ensure precise movement of the robot.

- Odometry Calculation: Real-time computation of the robot's position and orientation based on wheel encoder data.

- Localization: Integration with other sensors (if applicable) for accurate positioning within the environment.

- PID Control: Implementation of PID algorithms for precise motor control and trajectory correction.

- Modular Design: The software is designed to be easily extendable and modifiable to accommodate additional features or different robot configurations.

- ROS Integration: Includes multiple ROS nodes for publishing odometry and velocity data to other high-level nodes in the system, facilitating easy integration into larger robotic architectures.

- Two-Wheel Robot Support: The code is specifically optimized for a two-wheeled robot equipped with two encoders, ensuring precise differential drive control and accurate odometry.

Made with love by Sami FAHIM ❤
