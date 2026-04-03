# 🚁 Drone Fleet Simulator

A Java-based **multi-drone simulation and visualization project** combining control systems, collision avoidance, communication modeling, and a JavaFX desktop GUI.

Simulate a fleet of drones navigating a 3D environment — tracking assigned targets, avoiding collisions, respecting communication constraints, and logging performance metrics. Includes an interactive **JavaFX GUI** for simulation control and a **game mode** for manual drone missions.

---

## 📸 Screenshots

### Simulation Tab — Fleet in flight over farm field
![Simulation Tab](screenshots/simulation-tab.png)

### Close-up — Drone formation with neighbor links and spray overlay
![Formation Close-up](screenshots/formation-closeup.png)

### Game Tab — Manual drone mission (spraying mode)
![Game Tab](screenshots/game-tab.png)

### Mission Complete!
![Mission Complete](screenshots/mission-complete.png)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Architecture](#project-architecture)
- [How the Simulation Works](#how-the-simulation-works)
- [GUI Workflow](#gui-workflow)
- [Controls](#controls)
- [Outputs](#outputs)
- [Repository Structure](#repository-structure)
- [Configuration](#configuration)
- [How to Run](#how-to-run)
- [Known Limitations](#known-limitations)
- [Future Enhancements](#future-enhancements)
- [License](#license)
- [Author](#author)

---

## Overview

The project has two main parts:

**`DroneSimulator`** — The core simulation engine:
- drone physics and dynamics
- thrust and torque control
- collision avoidance
- communication-based neighbor detection
- environment bounds enforcement
- CSV/TXT logging

**`DroneSimulatorGUI`** — The JavaFX desktop application:
- interactive 2D visualization
- shared and per-drone target assignment
- camera controls and follow mode
- custom drone/background skins
- simulation control panel
- manual mission-based game mode

---

## Features

### Core Simulation
- Multi-drone fleet simulation
- Position and velocity tracking
- Attitude control using rotation matrices
- Gravity and aerodynamic drag
- Collision avoidance using repulsive forces
- Communication range and packet loss modeling
- Environment boundary handling
- CSV-based position logging
- TXT-based final metrics summary

### GUI
- JavaFX-based real-time simulation interface
- 2D fleet rendering with pseudo-3D altitude effects
- Shared and per-drone target assignment
- Random and zero-position initialization
- Pause, reset, and speed control
- Click-to-follow drone camera
- Pan and zoom controls
- Toggleable grid, labels, neighbor links, and spray overlays
- Load constants from external configuration file
- Custom drone and background images
- White-background removal for PNG drone assets

### Game Mode
- Manual drone movement with keyboard (`W A S D`)
- Mission target spawning
- Spray-based task completion (`SPACE`)
- Progress tracking and mission-complete popup with fireworks

---

## Project Architecture

### Core Simulation Package: `DroneSimulator`

| Class | Responsibility |
|---|---|
| `ClassMain` | Entry point; defines constants, creates drones, initializes all systems, starts the loop |
| `Drone` | Represents a single drone — stores physics state, handles force accumulation and integration |
| `Controller` | Position tracking control, velocity damping, gravity compensation, thrust and torque generation |
| `CollisionAvoidance` | Computes short-range repulsive forces; tracks unsafe inter-drone distance events |
| `CommunicationModule` | Models communication range and packet loss; handles neighbor discovery |
| `FormationManager` | Computes formation force from relative position and velocity errors |
| `Environment` | Defines world boundaries; clamps position and reverses velocity at edges |
| `Simulator` | Runs the simulation loop — reset, apply forces, integrate, log each step |
| `Logger` | Writes `Positions.csv` and `Metrics.txt` |
| `Vector3` | 3D vector math utility (add, subtract, multiply, normalize, dot, cross, distance) |
| `Matrix3D` | 3×3 matrix utility (identity, transpose, multiply, Rodrigues rotation, skew-symmetric) |

### GUI Package: `DroneSimulatorGUI`

| Class / File | Responsibility |
|---|---|
| `HelloApplication` | Main JavaFX launcher — loads FXML, creates the stage |
| `hello-view.fxml` | Defines the UI layout (Simulation tab + Game tab) |
| `DroneSimulatorController` | Connects UI to simulation logic; renders drones; handles camera, controls, and game mechanics |

---

## How the Simulation Works

### Drone Dynamics
Each drone is influenced at every step by:
- gravity
- thrust
- aerodynamic drag
- formation force
- repulsive collision-avoidance force

### Thrust Control
The controller computes desired acceleration from position error, velocity error, and gravity compensation — then converts it to a body-frame thrust vector.

### Torque Control
Drone orientation is updated by computing a desired rotation matrix from the target acceleration and yaw, comparing it against the current rotation, and applying corrective torque.

### Communication
A drone only treats another as a neighbor if it is within communication range **and** the packet is not randomly lost (modeled probabilistically).

### Collision Avoidance
When two drones are closer than `minSafeDistance`, a repulsive force is applied and the collision counter increments.

### Environment Bounds
When a drone hits a boundary, its position is clamped and its velocity is reversed on that axis.

### Simulation Loop (per step)
1. Reset forces
2. Apply gravity
3. Discover neighbors
4. Compute thrust and torque
5. Compute formation force
6. Compute repulsive force
7. Sum all forces
8. Integrate motion
9. Apply environment bounds
10. Log state

---

## GUI Workflow

### Simulation Mode
The Simulation tab lets you:
- choose the number of drones and simulation duration
- initialize drones randomly or at the origin
- assign a shared target or unique per-drone targets
- start, pause, and reset the simulation
- zoom and pan around the environment
- click a drone to follow it (Ctrl+Click to stop)
- toggle labels, neighbor links, spray, and grid overlays
- customize drone and background visuals
- load constants from a `.properties` file

> **Shared target behavior:** When a shared target is set, the GUI distributes drones into a circular slot pattern around the target — avoiding all drones stacking on a single point.

### Game Mode
The Game tab provides a simplified interactive mission:
- fly the drone using `W`, `A`, `S`, `D`
- hold `SPACE` to spray
- stay inside the target circle long enough to complete the mission
- fireworks and a mission-complete popup appear on success

---

## Controls

### Simulation Tab

| Action | Input |
|---|---|
| Start simulation | **Start** button |
| Pause simulation | **Pause** button |
| Reset simulation | **Reset** button |
| Pan camera | Mouse drag |
| Zoom in/out | Mouse wheel |
| Follow a drone | Click drone |
| Stop following | Ctrl + Click |

### Game Tab

| Action | Input |
|---|---|
| Move drone | `W` / `A` / `S` / `D` |
| Spray | Hold `SPACE` |
| Start mission | **Start Mission** button |
| New target | **Random Target** button |
| Reset game | **Reset** button |

---

## Outputs

### `Positions.csv`
Time-series flight data logged each simulation step:

```csv
Time,Drone ID,Px,Py,Pz,Vx,Vy,Vz,Tz
```

Useful for analysis, plotting, debugging, replay, and controller tuning.

### `Metrics.txt`
Final simulation summary including:
- number of drones and total steps
- average inter-drone spacing
- collision count and minimum safe distance
- communication success rate
- final positions, velocities, and speeds
- target positions and distance-to-target per drone

---

## Repository Structure

```text
Drone-Fleet-Simulator/
│
├── src/
│   ├── DroneSimulator/
│   │   ├── ClassMain.java
│   │   ├── Drone.java
│   │   ├── Controller.java
│   │   ├── Simulator.java
│   │   ├── CollisionAvoidance.java
│   │   ├── CommunicationModule.java
│   │   ├── FormationManager.java
│   │   ├── Environment.java
│   │   ├── Logger.java
│   │   ├── Matrix3D.java
│   │   └── Vector3.java
│   │
│   └── DroneSimulatorGUI/
│       ├── HelloApplication.java
│       ├── DroneSimulatorController.java
│       ├── hello-view.fxml
│       └── module-info.java
│
├── assets/
│   ├── drones/
│   └── backgrounds/
│
├── output/
│   ├── Positions.csv
│   └── Metrics.txt
│
├── screenshots/
│   ├── simulation-tab.png
│   ├── formation-closeup.png
│   ├── game-tab.png
│   └── mission-complete.png
│
├── config/
│   └── simulation.properties
│
└── README.md
```

---

## Configuration

The GUI can load a constants file via the **Load Constants File** button.

Example `simulation.properties`:

```properties
dt=0.01
totalTime=60
numDrones=4

mass=1.5
kd=0.2

Ix=0.02
Iy=0.02
Iz=0.04

kp=3.5
kdCtrl=2.5
kR=8.5
kOmega=0.15

formKp=0.0
formKv=0.0

kRep=1.0
commRange=50.0
packetLoss=0.01

envWidth=100.0
envHeight=100.0
envDepth=1000.0
minSafeDistance=1.0
```

> **Note:** `formKp` and `formKv` are set to `0.0` by default — formation force exists in the architecture but is inactive unless these gains are increased.

---

## How to Run

### Option 1: Run from an IDE (Recommended)

1. Import the project into IntelliJ IDEA or Eclipse.
2. Configure JavaFX in the project module settings.
3. Run the desired entry point:

| Mode | Entry Point |
|---|---|
| Console simulation | `DroneSimulator.ClassMain` |
| GUI application | `DroneSimulatorGUI.HelloApplication` |

### Option 2: Command Line

Requirements:
- JDK installed
- JavaFX SDK configured in your module path

Because this project uses JavaFX modules, command-line launch depends on your local JavaFX installation. Running from an IDE is recommended unless you already have a JavaFX CLI setup ready.

---

## Known Limitations

- The console `ClassMain` assigns `targetZ = 2000` while environment depth is `1000`, so drones will eventually be constrained by the environment ceiling.
- Communication logging may overcount failures since `logCommunication(false)` is called during neighbor checks even when a successful message is also recorded.
- Collision counting is cumulative and may overcount if drones remain within unsafe distance across multiple steps.
- Integration uses simple Euler-style steps and simplified dynamics — suited for educational and prototype use, not high-fidelity flight simulation.
- The GUI uses pseudo-3D visualization rather than real 3D rendering.
- No build tool configuration (Maven/Gradle) is included yet — setup is manual.

---

## Future Enhancements

- Add Maven or Gradle build support
- Export charts directly from logs
- Add obstacle fields and terrain
- Real 3D rendering
- PID tuning panel in GUI
- Improved communication statistics
- Refined collision metrics
- Save and load full simulation scenarios
- Replay mode from CSV logs
- Unit test coverage
- Demo GIF in repository

---

## License

This project is licensed under the [MIT License](LICENSE).

---

## Author

**Hassan Ali**  
Java | Simulation | Drone Systems | JavaFX

[![GitHub](https://img.shields.io/badge/GitHub-Hassan--Ali-181717?style=flat&logo=github)](https://github.com/harryhassan)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0A66C2?style=flat&logo=linkedin)](https://www.linkedin.com/in/hassanharry/)

