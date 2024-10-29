# Requirements

This section will hold the requirements for the project.

## Goal

The goal is to program an autonomous vehicle to drive at least one lap on an indoor track in a fast, yet safe manner.

## Assumptions

* The vehicle is placed in driving direction.
* F1TENTH Cone System: Left cones are blue, right cones are yellow, the start zone has orange cones on both sides.
* There will be no other vehicles on the track.
* The maximum speed is around 15 km/h.
* The race car is ROS based.
* Hardware used:
	* 3D + Mono Camera (Intel RealSense Depth Camera D435i)
	* High-Resolution 2D LiDAR (Hokuyi UST-10 LiDAR)
	* NVIDIA Jetson NX 16GB
	* 4 Wheel Drive with Ackerman Steering
* The vehicle will be driving on an indoors track.
* The track is a loop and doesn't have any branching.
* Perception round and then timed round.
* Track layout changes between races.

## Functional

This section will hold the functional requirements for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | F1 | General Perception | The vehicle shall capture real-time 3D depth data using the Hokuyo UST-10 LiDAR and high-resolution visual data using the Intel RealSense Depth Camera D435i to perceive the environment. | Perception
2 | F2 | Object Detection & Classification | The system shall detect and classify blue cones marking the left track boundary and yellow cones marking the right track boundary in real-time. | Perception
1 | F1 | Cone Detection | When on track the vehicle shall detect cones marking the track boundaries in real-time using the camera and LiDAR sensors. | Perception
3 | F2 | Sensor Fusion | When collecting data the perception system shall integrate LiDAR and camera data using a sensor fusion algorithm to improve cone detection accuracy. | Perception
4 | F4 | Distance Estimation | While driving the vehicle shall estimate the disctance to the track boundaries continuously. | Perception
4 | F4 | Point Cloud | The vehicle shall collect 3D LiDAR point cloud data to detect and map its surroundlings in real-time. | Perception
5 | F5 | Visual Data | The camera system shall provide input for visual odometry and SLAM, ensuring accurate self-localization. | Perception, Localization
5 | F5 | Self-Localization | The vehicle shall localize its position relative to track boundaries using fused data from LiDAR, camera, and odometry. | Localization, Perception
5 | F5 | Real-Time Map | The vehicle shall generate a real-time map of the environment using LiDAR and visual sensors, aiding in cone detection and path planning. | Perception, Planning, Localization
6 | F6 | Environments | The system shall be able to handle both static and dynamic environments, where the track layout may change.
7 | F7 | Optimal trajectory | When planning the path the vehicle shall generate an optimal trajectory within track boundaries using the detected cone positions and real-time sensor data, maintaining the desired speed. | Planning
7 | F7 | Path Planning | The vehicle shall implement a planning algorithm that works in real time, adjusting paths as the environment changes. | Planning
7 | F7 | Path Updates | The path planner shall compute new trajectories dynamically to handle sudden track changes. | Planning
9 | F9 | Acceleration & Deceleration | The vehicle shall decide when to accelerate, decelerate, or stop, depending on the environment. | Perception, Planning, Control
9 | F9 | Driving Modes | The vehicle shall be able between different driving modes (e.g. cruising, emergency stop) based on the context. | Planning, Control, Monitoring
9 | F9 | Speed Maintenance | The vehicle shall maintain the fastest appropriate speed for the calculated paths on the track layout, not exceeding 15 km/h. | Planning
9 | F9 | Acceleration Adaptation | The system shall adapt acceleration to keep the vehicle within speed limits, adjusting for factors like road slope, friction, and other dynamic conditions. | Control
10 | F10 | Sharp Turns | While approaching sharp turns the vehicle shall adjust its speed to ensire safe cornering without exceeding the track boundaries. | Planning, Control
11 | F11 | Smooth Acceleration | When accelreating the vehicle mist limit its accelaration to avoid jerky movements using a sof acceleration curve, ensuring smooth control. | Control
12 | F12 | Braking | The vehicle shall be able to brake in real-time to slow down or stop based on planned behavior or unexpected events. | Control
12 | F12 | Controlled Emergency Braking | When braking the vehicle shall decelerate within a controlled distance to avoid overshooting turns or track boundaries. | Control
13 | F13 | Steering Angle | When controlling the steering the vehicle shall calculate the optimal steering angle based on the track curvature and current speed. | Control
13 | F13 | Steering | The vehicle shall control the Ackerman steering system to follow the planned trajectory accurately. | Control
13 | F13 | Smooth Steering | The control system shall ensure stable and smooth steering, especially at high speeds. | Control
15 | F15 | External Signals | The system shall respond appropriately to external signals and traffic rules defined for the track. | Control, Perception, Planning
15 | F15 | Failsafe Mode | When a critical system failure is detected the vehicle shall engage the emergency braking system to bring the vehicle to a stop. | Monitoring, Control
16 | F16 | System Health | When system health is checked the vehicle shall monitor the status of all sensors, compute hardware, and acutators in real-time to ensure continuous operation. | Control
16 | F16 | Odometry | The vehicle shall collect odometry data to track its speed, orientation and position over time. | Control, Monitoring
16 | F16 | Health Checks | The system shall implement watchdog timers and health-check nodes ro detect failures or malfunctions in any part of the system. | Monitoring
17 | F17 | Vehicle Tracking | The system shall provide continuous and real-time feedback on the vehicle's movement and position to maintain trajectory tracking. | Monitoring, Planning, Control
18 | F18 | Internal Communication | All sensors, acutators, and the onboard compute hardware shall communicate via ROS2 Humble, using DDS for real-time message passing. | Communication
19 | F19 | Manual Override Mode | The system should be able to transition to a manual override mode in case of critical failures, allowing the race team to take control if neccessary. | Monitoring, Control
19 | F19 | Partial System Failures | The system shall include mutliple levels of safety checks, and emergency overrides shall be functional even in the event of partial system failures. | Control, Monitoring
20 | F20 | Testing | The vehicle shall be tested in both simulation and real-world environments to validate its perception, planning, and control systems. | Testing

## Quality

This section will hold the quiality requirements for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | Q1 | Sensor Processing Latency | The system shall process sensor data in real-time with latency under 50 ms to ensure timely decision-making and control. | Decision Making, Control
2 | Q2 | Control Loop Frequency | The vehicle shall maintain a control loop update frequency of 20-30Hz to ensure smooth and responsive driving at max. 15 km/h. | Control
3 | Q3 | Localization Accuracy | The localization system shall maintain position ±10 cm to ensure the vehicle stays within the track boundaries. | Localization
4 | Q4 | Perception Radius | The perception system shall detect cones aat least 10 meters ahead to provide sufficient time for path adjustments. | Perception
5 | Q5 | Emergency Braking Distance | The vehicle shall decelerate within 2-3 meters when the emergency braking system is engaged to ensure safe stopping. | Monitoring, Control, Safety
6 | Q6 | Emergency Stop | The vehicle shall engage the emergency braking system within 100 ms of detecting critical system failures to avoid accidents. | Control, Monitoring
8 | Q8 | Detection Rate | The perception system shall maintain a continuous detection rate of at least 95% for cones to prevent boundary violations. | Perception
2 | Q2 | Detection & Classification Range | The system shall detect and classify blue cones marking the left track boundary and yellow cones marking the right track boundary in real-time within a range of at least 2 m. | Perception
9 | Q9 | Control System | The control system shall operate continuously for the duration of the race without any system crashes or interruptions. | Monitoring, Control
10 | Q10 | Fallback | The vehicle shall recover from sensor or communication failures within 200 ms using fallback systems to ensure continuous operation. | Monitoring
11 | Q11 | Logs | The system shall log sensor and control data with timestamps for a post-race analysis without exceeding storage limits. | Monitoring
12 | Q12 | Hardware Temperature | The compute hardware shall maintain stable operation withing a temperature range of 0-50°C to prevent overheating or shutdowns. | Monitoring
13 | Q13 | Real-Time Data | The system shall provide real-time telemetry data to the race team at least once every 100 ms to allow for remote monitoring during the race. | Monitoring
14 | Q14 | Reconfiguration & Updates | The system shall support easy reconfiguration and updates of perception and control algorithms without requiring major code overhauls. | Perception, Control
15 | Q15 | Diagnostics | The user interface shall allow for quick diagnostic checks of system health before and during the race to facilitate rapid troubleshooting. | Monitoring
16 | Q16 | Real-Time Internal Communication | The system shall support reliable, low-latency communication between sensor nodes, perception nodes, and control nodes to ensure real-time response. | Communication
17 | Q17 | Energy Consumption | The software stack should optimize CPU and GPU usage to minimize energy consumption during intensive tasks like real-time perception and path planning. | Perception, Planning
18 | Q18 | Sensor Efficiency | The vehicle shall use its sensors efficiently, ensuring that data redundancy is minimizes, and sensor processing loads are balances to avoid system bottlenecks. | Perception
19 | Q19 | Data Fitlering | The system should implement intelligent data filtering to reduce unnecessary sendor data while preserving critical information needed for accurate perception and planning. | Perception, Planning
20 | Q20 | Trajectory Updates | The path planning system shall generate new trajectories within a strict time frame/ every 50 ms to adapt to changes in the environment, avoiding delays that could cause the vehicle to miss turns. | Planning

## Constraints

This section will hold the constraints for the project.

\# | ID | Name | Text | Autonomous Driving Task(s)
---|----|------|------|---------------------------
1 | C1 | Speed Limit | The vehicle shall not exceed a speed of 15 km/h due to track limitations and safety considerations. | Control
2 | C2 | Jetson Limitiations | The system shall operate within the limitations of the NVIDIA Jetson NX ensuring all processes can run in real-time without overloading the CPU, GPU, and memory resources. | Perception, Planning, Control, Monitoring
3 | C3 | Real-Time Performance | The system shall process sensor inputs at 20-30Hz to maintain real-time performance within the computational limits of the vehicle. | Perception
4 | C4 | Remote Emergency Stop | The vehicle shall include a Remote Emergency Stop system that can be triggered by the race team/ race officials at any time during the race. | Control
5 | C5 | System Monitoring | The autonomous system shall continuously monitor critical safety components and transition to a safe state if a failure is detected. | Monitoring
6 | C6 | Power Budget | The sensor suite shall operate within a power budget defined by the F1TENTH platform, ensuring no component exceeds the power capacity available to the vehicle. | Monitoring
7 | C7 | Ackerman Steering | The vehicle shall use the Ackerman steering system for turning, which constains the types of maneuvers the vehicle can perform, preventing mechanical failure or loss of control, especially at high speeds. | Control, Planning
8 | C8 | On Track Testing | Vehicle testing on track shall be done within the practice tracks provided. | Testing
10 | C10 | Processor Limitations | Algorithms for perception, planning, and control shall be designed to avoid overloading the processor, ensuring real-time performance without exceeding the power and thermal limits of the hardware. | Perception, Planning, Control
12 | C12 | 4-Wheel Drive | The vehicle's control system shall optimize the distribution of torque across all four wheels in a manner that ensures both optimal acceleration and stability, without causing skidding or loss of traction. | Control
13 | C13 | Energy Consumption | The vehicles's energy consumption shall be managed to ensure that it can complete the entire race or testing session within the available battery capacity, taking into account the energy demands of high-speed racing and intensice computations. | Perception, Planning, Control, Monitoring
14 | C14 | Data Priorization | The system shall ensure that critical sensor data is prioritized in high-speed scenarions, where rapid decision-making is essential.
