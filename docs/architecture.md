``` mermaid
graph LR
    RoutePlanning[Route Planning] -->|Target per GUI| MissionManeuver[Mission & Maneuver Planning]

    subgraph Planning
      MissionManeuver -->|Traffic Rules| RoutePlanning
      MissionManeuver -->|Global Map| PathPlanning[Path Planning]
      PathPlanning -->|Trajectory| PathControl[Path Control]
    end

    subgraph Control
      PathControl -->|Ackermann Dem| VehicleControl[Vehicle Control]
      VehicleControl -->|Steering CMDs| VehicleInterface[Vehicle Interface]
      VehicleInterface -->|CAN| Vehicle[Vehicle]
    end 
    Vehicle --> Traffic[Traffic]

    Localization -->|Pose| PathControl
    Localization --> MissionManeuver

    SensorDataProcessor[Data-Pre-Processing] -->|Objects: lanes, lights, signs, etc.| SensorFusion
    SensorDataProcessor -->|Objects: free space, lanes, objects, etc.| SensorFusion
    SensorDataProcessor -->|Objects: active vehicles, active infrastructure, etc.| SensorFusion

    SensorFusion -->|Cost Map including all objects| MissionManeuver
    MissionManeuver -->|Weighted Cost Map| PathPlanning

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