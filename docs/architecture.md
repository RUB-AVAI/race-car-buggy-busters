This architecture outlines a high-level design for the autonomous vehicle system, organized into four main subsystems: Sensing, Localization, Planning, and Control.<br>
**Sensing:** Combines sensors and data preprocessing.<br>
**Localization:** Manages vehicle positioning and map data integration.<br>
**Planning:** Includes route and maneuver planning.<br>
**Control:** Handles direct vehicle controls.<br>
In this architecture, each subsystem communicates through well-defined data flows, with Localization and Planning feeding into Control, while Sensing provides real-time environmental context.

``` mermaid
architecture-beta
    group perception[Perception]
    
    service localization[Localization SLAM] in perception
    service sensorFusion[Sensor Fusion] in perception
    service sensorDataProcessor[PreProcessing] in perception
        

    group sensing[Sensing]
    service camera[Camera] in sensing
    service lidar[Lidar] in sensing
    service v2x[V2X] in sensing
        
    
    group planning[Planning]
    service routePlanning[Route Planning] in planning
    service missionManeuver[MissionAndManeuverPlanning] in planning
    service pathPlanning[PathPlanning] in planning
    service pathControl[PathControl] in planning
        
    group control[Control]
    service vehicleControl[VehicleControl] in control
    service vehicleInterface[VehicleInterface] in control
    service vehicle[Vehicle] in control

    service watchdog[WatchDogTimer]
    service traffic[Traffic]


    %%camera:R -- L:sensorDataProcessor
    %%camera:R -- L:localization
    %%lidar:R -- L:sensorDataProcessor
    %%v2x:T -- B:missionManeuver
    %%sensorFusion:B -- T:missionManeuver
    %%pathControl:L -- R:vehicleControl
    vehicle:B -- T:traffic
    vehicleInterface:B -- T:watchdog 
    pathPlanning:B -- T:pathControl
    vehicleControl:B -- T:vehicleInterface
    vehicleInterface:L -- R:vehicle
    sensorDataProcessor:T -- B:sensorFusion
    missionManeuver:R -- L:pathPlanning
    missionManeuver:L -- R:routePlanning
```
``` mermaid
graph LR
    RoutePlanning[Route Planning] -->|Target per GUI| MissionManeuver[Mission & Maneuver Planning]

    subgraph Planning
      MissionManeuver -->|Traffic Rules| RoutePlanning
      MissionManeuver -->|Weighted Cost Map| PathPlanning[Path Planning]
      PathPlanning -->|Trajectory| PathControl[Path Control]
    end

    subgraph Control
      PathControl -->|Ackermann Dem| VehicleControl[Vehicle Control]
      VehicleControl -->|Steering CMDs| VehicleInterface[Vehicle Interface]
      VehicleInterface -->|CAN| Vehicle[Vehicle]
    end

    Vehicle --> Traffic[Traffic]
    Watchdog[Watchdog Timer] --> VehicleInterface
    VehicleInterface --> Watchdog
  

    Localization[Localization SLAM] -->|Pose| PathControl
    Localization --> MissionManeuver

    SensorDataProcessor[Data-Pre-Processing] -->|Objects: lanes, lights, signs, etc.| SensorFusion
    SensorDataProcessor -->|Objects: free space, lanes, objects, etc.| SensorFusion
    SensorDataProcessor -->|Objects: active vehicles, active infrastructure, etc.| SensorFusion
    SensorFusion -->|Cost Map including all objects| MissionManeuver

    subgraph Perception
      Localization
      SensorFusion
    end

    subgraph Sensing
      Camera --> SensorDataProcessor
      Lidar --> SensorDataProcessor
      V2X --> SensorDataProcessor
    end
      
    Camera --> Localization
    Lidar --> Localization
    V2X --> |Infrastructure| MissionManeuver
    
    Localization -->|HD Map| PathPlanning
```
