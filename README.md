# 🏁 F1 Racing Simulator

A professional-grade **Formula 1 racing simulator** built with **Unity 3D**, featuring advanced physics simulation with Pacejka tire models, realistic aerodynamics, sophisticated braking systems, and responsive vehicle dynamics. Designed for high-performance racing gameplay with authentic vehicle behavior.

## 📋 Overview

This is a **feature-rich racing simulation engine** that prioritizes physics accuracy and realistic vehicle behavior. The project implements industry-standard tire modeling, advanced suspension systems, dynamic aerodynamics, and intelligent braking mechanics to deliver an authentic F1 racing experience.

**Key Philosophy**: Every system is modular, tunable via ScriptableObject profiles, and designed for both simulation accuracy and gameplay responsiveness.

---

## 🎯 Core Features

### 🏎️ **Advanced Physics System**
- **Raycast-based wheel suspension** with independent spring/damper dynamics
- **Pacejka tire model** implementation for accurate lateral and longitudinal grip
- **Load-sensitive grip calculation** with speed-band multipliers
- **Real-time axle load transfer** and weight distribution
- **Friction ellipse** modeling for combined lateral/longitudinal forces
- **Dynamic aerodynamic downforce** with front/rear bias distribution

### 🛞 **Wheel & Suspension**
- Independent suspension for each wheel (FL, FR, RL, RR)
- Configurable spring stiffness, damper strength, and rest length
- Ground contact detection with multi-raycast fallback system
- Suspension travel tracking and settlement timing
- Visual wheel spin feedback synchronized with physics
- Automatic settling on startup with anchor distance correction

### 🔧 **Drivetrain & Braking**
- **Rear-wheel drive** with configurable power distribution
- Speed-dependent motor torque curves
- **Advanced braking system** with:
  - Virtual brake pedal pressure modeling
  - Front/rear brake bias with dynamic adjustment
  - Brake lockup detection and efficiency loss
  - Late-braking multiplier (increased braking force at speed)
  - Trail braking support for cornering with throttle modulation
  - Rear instability simulation during braking on turns

### 🎛️ **Steering & Traction Control**
- **Speed-sensitive steering** with customizable response curves
- **Ackermann geometry** implementation for realistic turning
- **Advanced steering assist** with three assistance levels:
  - Oversteer correction (power slides)
  - Understeer mitigation (tight turns)
  - Four-wheel slide detection and recovery
- Real-time **traction control** preventing wheel slip
- Slip angle monitoring and countersteer recommendations

### 🌬️ **Aerodynamics**
- **Downforce generation** scales with speed (quadratic)
- Configurable front/rear downforce bias (38% default front)
- **Lateral drag** for high-speed stability
- Onset speed thresholds for aerodynamic effects
- Pressure capping to prevent unrealistic forces

### 📊 **Real-Time Telemetry & Debugging**
- **Live physics debug panel** with on-screen visualization:
  - Vehicle speed (km/h and m/s)
  - Wheel grip utilization bars with color coding
  - Axle loads and weight transfer G-forces
  - Suspension compression per wheel
  - Brake lockup indicators
  - Steering assist angles
- Runtime parameter sliders for live tuning
- Gizmo visualization for suspension, contact points, and slip angles

### 🎥 **Dynamic Camera**
- **Speed-dependent FOV adjustment** (base 60° → max 72° at 260 km/h)
- Smooth FOV transitions for immersive acceleration feel
- Cinemachine integration support
- Fallback to standard Unity camera

### 📱 **Input System**
- Keyboard and controller support via Unity Input System
- Mobile touch control integration (steering, throttle, brake)
- Seamless external input API for AI/replay systems
- Configurable input mappings

### 🎨 **Visual Body Alignment**
- Real-time visual chassis orientation from suspension compression
- **Pitch adjustment** from front/rear suspension load transfer
- **Roll correction** from lateral weight transfer
- High-speed aero pitch and drop effects
- Smooth interpolation between poses

---

## 🛠️ Tech Stack

| Component | Language | Purpose |
|-----------|----------|---------|
| **Core Physics** | C# (65.8%) | Game logic, physics simulation, vehicle systems |
| **Graphics/Shaders** | ShaderLab (25%) | Post-processing, material rendering, visual effects |
| **Low-Level Graphics** | HLSL (5.2%) | DirectX-level graphics optimization |
| **Tools/Analysis** | Wolfram Language (4%) | Data analysis and script generation |

**Engine**: Unity 3D (2020.3 LTS or later)  
**Physics**: Custom raycast-based system with Pacejka tire modeling  
**Input**: Unity Input System (new) with mobile support  

---

## 📁 Project Structure

```
Assets/
├── Scripts/
│   └── Brain/                          # Core physics and vehicle systems
│       ├── VehiclePhysicsCoordinator.cs    # Master physics orchestrator (-100 execution order)
│       ├── RaycastWheel.cs                 # Suspension & wheel physics
│       ├── TractionSystem.cs               # Pacejka tire model & grip
│       ├── DrivetrainBrakeSystem.cs        # Motor & basic braking
│       ├── AdvancedBrakingSystem.cs        # Advanced brake logic, lockup, trail braking
│       ├── SteeringSystem.cs               # Steering input, Ackermann, assist
│       ├── AdvancedSteeringAssist.cs       # Traction control & oversteer/understeer correction
│       ├── DownforceSystem.cs              # Aerodynamic forces
│       ├── WeightTransfer.cs               # Axle load and G-force calculations
│       ├── CameraSpeedPerception.cs        # Dynamic FOV based on speed
│       ├── PhysicsDebugPanel.cs            # Real-time telemetry UI
│       ├── WheelVisual.cs                  # Wheel mesh animation
│       └── BasicMotor.cs                   # Legacy drivetrain wrapper
│
├── Materials/                          # Physics materials and shaders
├── Prefabs/                            # Pre-configured vehicle setups
├── Profiles/                           # ScriptableObject tuning profiles
├── Scenes/                             # Game scenes
├── Settings/                           # Project configuration
├── Resources/                          # Runtime-loaded assets
├── Tests/                              # Unit tests
├── Docs/                               # Documentation
│
├── InputSystem_Actions.inputactions    # Input bindings configuration
├── ProjectSettings/                    # Unity project settings
├── Packages/                           # Project dependencies
└── Android_Builds/                     # Android build outputs
```

---

## 🚀 Getting Started

### Prerequisites

- **Unity 2020.3 LTS** or later (tested on 2022+)
- **Windows, Mac, or Linux** development environment
- **Visual Studio** or preferred C# IDE
- **3GB+ disk space** for project files

### Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/Aakashh-01/F1.git
   cd F1
   ```

2. **Open in Unity**
   - Launch **Unity Hub**
   - Click **"Open Project"** (or add to your library)
   - Navigate to the cloned F1 folder
   - Wait for project to load and compile (~2-3 minutes first time)

3. **Verify Setup**
   - Open `Assets/Scenes/` and locate the main scene
   - Ensure all script references resolve (no missing components)
   - Run the scene to verify physics initialization

### First Run Checklist

- ✅ Rigidbody on car is configured (mass ~1200kg, drag ~0.02)
- ✅ Four wheels named: `FL`, `FR`, `RL`, `RR`
- ✅ Wheels have `RaycastWheel` components
- ✅ All systems referenced in `VehiclePhysicsCoordinator`
- ✅ Ground layer properly configured in wheel raycasts
- ✅ Scene has proper lighting and terrain

---

## 🎮 Gameplay Guide

### Controls

| Action | Keyboard | Controller |
|--------|----------|-----------|
| **Accelerate** | `W` / Up Arrow | `RT / Right Trigger` |
| **Brake** | `S` / Down Arrow | `LT / Left Trigger` |
| **Steer Left** | `A` / Left Arrow | `Left Stick X` |
| **Steer Right** | `D` / Right Arrow | `Left Stick X` |
| **Reverse** | `S` when stationary | `LT` when stopped |
| **Handbrake** | `Space` | `RB / Right Bumper` |
| **Debug Panel** | `Tab` | `Y` |
| **Camera Toggle** | `C` | `X` |

### Game Modes (To Be Implemented)

- **Time Trial**: Compete against the clock on a single lap
- **Race**: Full-length F1 races with AI opponents
- **Practice**: Free practice sessions with unlimited fuel
- **Career**: Multi-season progression with team management

---

## 🎛️ Physics System Deep Dive

### Tire Model: Pacejka Magic Formula

The **TractionSystem** uses the Pacejka tire model to calculate grip forces:

```
Grip Force = D × sin(C × arctan(B×x - E×(B×x - arctan(B×x))))
```

Where:
- **B** (Stiffness): Controls grip response to slip
- **C** (Shape): Determines curve shape
- **D** (Peak): Maximum achievable grip
- **E** (Curvature): Fine-tunes the drop-off

**Load Sensitivity**: Grip scales with load (normal force) raised to a power (0.68 default):
```
Effective_Peak = D × (Normal_Force / Reference_Load)^0.68
```

**Speed Bands**: Grip multipliers adjust with speed:
- **Low Speed (0-15 km/h)**: 82% grip (better control at low speeds)
- **Mid Speed (15-60 km/h)**: 100% grip (nominal)
- **High Speed (60+ km/h)**: 88% grip (aerodynamic degradation)

### Suspension System

Each wheel independently calculates:
1. **Spring Force** = Static Weight + Stiffness × Displacement
2. **Damper Force** = Damper Constant × Suspension Velocity
3. **Normal Force** = Spring Force + Damper Force (clamped to max)

### Steering Assist Levels

| Level | Oversteer | Understeer | Slip Threshold |
|-------|-----------|------------|-----------------|
| **Low** | 0.15 | 0.05 | 15° |
| **Medium** | 0.30 | 0.15 | 8° |
| **High** | 0.45 | 0.25 | 5° |

---

## 📊 Customization & Tuning

### Profile System

All vehicle parameters are stored in **ScriptableObject profiles** for easy switching:

1. **Create a Profile**:
   - Right-click in Project → Create → Vehicle Physics Profile
   - Configure tire grip, suspension, steering, aero, drivetrain parameters

2. **Apply to Vehicle**:
   - Drag profile into `VehiclePhysicsCoordinator.physicsProfile`
   - Check `applyProfileOnAwake`

3. **Live Tuning**:
   - Press `Tab` to open debug panel
   - Adjust sliders in real-time
   - Changes apply immediately

### Key Tuning Parameters

**Tire Grip** (`TireGripProfile`):
- Pacejka B, C, D, E coefficients (lateral & longitudinal)
- Load sensitivity (how grip scales with weight)
- Speed band multipliers

**Suspension** (`SuspensionProfile`):
- Spring stiffness (N/m)
- Damper strength (N·s/m)
- Suspension length (meters)
- Rest length ratio (0-1)

**Steering** (`SteeringAssistProfile`):
- Max steer angle (degrees)
- Speed sensitivity curve
- Ackermann factor
- Assist strength levels

**Aerodynamics** (`AeroProfile`):
- Downforce coefficient
- Front/rear bias
- Lateral drag coefficient
- Drag onset speed

**Braking** (`AdvancedBrakeProfile`):
- Brake pressure curves
- Lockup thresholds
- Trail braking strength
- Rear instability parameters

---

## 🐛 Debugging & Troubleshooting

### Physics Debug Panel

Press **Tab** (or Y on controller) to open the telemetry overlay showing:
- Real-time vehicle speed
- Per-wheel grip utilization (color-coded: green = good, red = slipping)
- Suspension compression per wheel
- Brake lockup indicators
- Steering assist angles
- Runtime parameter sliders

### Common Issues

| Issue | Solution |
|-------|----------|
| **Car doesn't move** | Check motor force is > 0; verify wheels are grounded |
| **Wheels clip through ground** | Increase raycast probe height; check layer masks |
| **Unstable physics** | Reduce spring stiffness; increase damper; lower timestep |
| **Sliding off road** | Increase tire grip coefficients; reduce downforce |
| **Camera jittering** | Reduce FOV smoothing time; check frame rate consistency |

### Gizmo Visualization

In Scene view with Gizmos enabled:
- **Green lines**: Suspension compression
- **Yellow arrows**: Slip vectors
- **Red sphere**: Ground contact points
- **White circles**: Wheel visual position

---

## 🔧 Advanced Features

### External Input API

Use the coordinator's `SetExternalInput()` for AI, replays, or networked control:

```csharp
coordinator.UseExternalInput = true;
coordinator.SetExternalInput(steeringInput, throttle, brakeInput);
```

### Force & Torque Queueing

All physics subsystems queue forces/torques through the coordinator:

```csharp
coordinator.QueueForceAtPosition(force, position, ForceMode.Force);
coordinator.QueueTorque(torque, ForceMode.Force);
```

This ensures deterministic order and prevents race conditions.

### Custom Physics Profiles

Extend `VehiclePhysicsProfile` to add custom data:

```csharp
[CreateAssetMenu(menuName = "Vehicle/Custom Profile")]
public class CustomProfile : VehiclePhysicsProfile 
{
    public float customTurboBoost = 1.2f;
}
```

---

## 📈 Performance Optimization

- **Raycast pooling**: Reuses arrays to reduce GC allocations
- **Execution order**: Physics runs at -100 for proper coordination
- **Force batching**: Queues forces before applying to rigidbody
- **Suspension settling**: 8-frame stability check prevents oscillation
- **LOD suspension fading**: Visual suspension effects fade at high speed

---

## 🤝 Contributing

Contributions are welcome! Areas for enhancement:

- [ ] AI opponent racing
- [ ] Multi-player networking (Netcode/Fusion)
- [ ] Track creation tools
- [ ] Advanced telemetry recording/replay
- [ ] Sound effects and music integration
- [ ] UI/HUD system
- [ ] Car customization/livery editor
- [ ] Performance profiling suite

**How to contribute:**

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/YourFeature`
3. Commit changes: `git commit -am 'Add YourFeature'`
4. Push to branch: `git push origin feature/YourFeature`
5. Open a Pull Request with detailed description

---

## 📝 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

You are free to use, modify, and distribute this code for personal and commercial projects.

---

## 👤 Author & Credits

**Aakashh-01** — [GitHub Profile](https://github.com/Aakashh-01)

### Acknowledgments

- **Pacejka Tire Model** inspired by established racing simulation research
- **Unity Community** for engine and plugin support
- **Formula 1** for inspiring competitive racing mechanics
- All contributors and testers who helped refine the physics

---

## 📞 Support & Feedback

For issues, feature requests, or questions:

- 🐛 **Bug Reports**: [GitHub Issues](https://github.com/Aakashh-01/F1/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/Aakashh-01/F1/discussions)
- 📧 **Contact**: Reach out via GitHub

---

## 📚 Additional Resources

- [Pacejka Tire Model Reference](https://en.wikipedia.org/wiki/Tire_models#Pacejka_model)
- [Unity Physics Documentation](https://docs.unity3d.com/Manual/PhysicsSection.html)
- [Racing Game Physics Guides](https://www.gamedev.net)
- [Project Documentation PDF](./Unity%20Game%20-%20Race%20Game%20-%20Mohammad.pdf)

---

**🏎️ Ready to race? Clone the repo, open it in Unity, and hit the track! 🏁**

*Last Updated: June 2026*
