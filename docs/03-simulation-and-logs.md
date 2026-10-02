# 03: Simulation, AdvantageScope, logs and tests

With only 4 hours of robot time a week, the plan is: **write and test on the laptop, use the
robot to confirm, and review the logs at home.**

## Run the robot in simulation

1. Open this folder in **WPILib VS Code**.
2. `Ctrl+Shift+P` → **WPILib: Simulate Robot Code**.
3. When it asks which extensions to use, tick **Sim GUI**. (The GUI is off by default in
   `build.gradle` because log replay needs every extension off.)
4. In the Sim GUI:
   - Plug in an Xbox controller and drag it from **System Joysticks** onto **Joysticks** slot 0.
     A keyboard works too.
   - Pick the alliance under **FMS** (Blue 1 is easiest to start with).
   - Set **Robot State** to **Teleoperated** to drive, or **Autonomous** to run the selected auto.

The simulated robot uses the same code as the real one, with physics models for the swerve
modules and flywheel, and PhotonVision's camera simulator standing in for the Limelight.

## Watch it in AdvantageScope

AdvantageScope comes with WPILib (Start menu → "AdvantageScope (WPILib)").

1. **File → Connect to Simulator** (or **Connect to Robot** at the robot).
2. Add a **3D Field** tab and pick the **2026** field. Then drag in:
   - `RealOutputs/Odometry/Robot` → the robot
   - `RealOutputs/Vision/Summary/TagPoses` → the tags the camera currently sees
   - `RealOutputs/Vision/Summary/RobotPosesAccepted` → vision's pose guesses (ghost robot)
   - `RealOutputs/Odometry/Trajectory` → the PathPlanner path being followed
   - `RealOutputs/Aim/TargetPosition` → the point being aimed at
3. Add a **Line Graph** tab with `RealOutputs/Aim/ErrorDegrees`,
   `RealOutputs/Shooter/TargetRPM`, `Shooter/LaunchVelocityRpm`, and
   `RealOutputs/Vision/Camera0/DetectionRatePercent`.
4. **File → Export Layout** to save it, so you get the same view every session.

> **Naming:** values the code *calculates* (`Logger.recordOutput`) show up under `RealOutputs/`.
> Sensor *inputs* (like `Shooter/LaunchVelocityRpm`) and `NetworkInputs/Tuning/...` show up at
> the top level. The docs drop the `RealOutputs/` prefix to keep things short.

### Changing tunable values live

Everything under `/Tuning/` (aim gains, shooter gains, manual RPM, speed limit) can be edited
while the code runs:

- **AdvantageScope:** turn on **tuning mode** (the slider icon in the sidebar), then click a value
  under `Tuning` and type.
- **Elastic** (also installed with WPILib): add a number widget for the topic and edit it.

Dashboard values are **not saved** to the code. Copy the winners back into the Java defaults.

### Picking an auto

The chooser is published as `SmartDashboard/Auto Choices`. Pick it from **Elastic**, or from the
Sim GUI's NetworkTables / SmartDashboard view. Then switch to Autonomous.

## What simulation is good for, and what it isn't

**Good for:** checking logic and wiring, aiming behavior, button bindings, PathPlanner autos and
pathfinding, shooter control, learning AdvantageScope, and catching crashes before robot day.

**Not good for:** judging vision *accuracy*. In this template the simulated camera "sees" the
world from the robot's *estimated* pose, so vision always agrees with odometry. Real tag detection
problems (lighting, blur, print quality) only show up on the real camera; that's what the bench
test is for. A physics library called **maple-sim** can add a separate "true" pose later.

## Automated tests

```powershell
./gradlew test
```

The results report is at `build/reports/tests/test/index.html`. The tests in
`src/test/java/frc/robot/SimulatedRobotTest.java` boot the whole robot in simulation with
simulated time and check real behavior:

| Test | Checks |
| --- | --- |
| `level2AimTurnsToFaceTheHub` | Starting 150° off, the robot turns to face the hub within 2° |
| `level2AimStaysOnTargetWhileStrafing` | The aim stays within 4° while strafing past the hub |
| `level1TxAimWorksDespiteLatencyAndDroppedFrames` | tx aim settles within 1.5° with 60 ms latency and **50% dropped frames** |
| `shooterReachesTargetSpeed` | The flywheel reaches 3000 RPM in under 2 s |
| `examplePathEndsAtTheShootingSpot` | The example path ends within 10 cm of the shooting spot |
| `pathfindingDrivesAroundTheHubToTheShootingSpot` | Pathfinding from behind the hub goes **around** it and reaches the spot |
| `autoChooserLoadedTheExampleAuto` | The .auto file loads |

Run the tests after every change. If one fails, the change broke *behavior*, which is a lot
cheaper to find at home than at the robot.

## Logs from robot sessions

The real robot logs **everything**, every loop.

1. Put a **USB stick** in the roboRIO before the session. Logs are written to `/U/logs`.
2. Afterwards, plug the stick into your laptop, or use **AdvantageScope → File → Download Logs**
   while connected to the robot.
3. Open the `.wpilog` in AdvantageScope with your saved layout.

What to look at:

- **Tag dropping out?** `Vision/Camera0/DetectionRatePercent` and `HasTarget` over time, next to
  what the robot was doing (turning fast? far away?). Pair it with the Limelight Rewind videos
  (Back button).
- **Aim wobbling?** `Aim/ErrorDegrees` and `Aim/OmegaRadPerSec`. Regular back-and-forth swings
  mean kP is too high or kD too low.
- **Pose jumping?** `Vision/Camera0/RobotPosesRejected` and `Accepted` on the 3D field.
- **Shooter inconsistent?** `Shooter/LaunchVelocityRpm` against `Shooter/TargetRPM` during a shot.

### Replay (advanced)

Replay feeds a real log back through *modified* code. Set `simMode = Mode.REPLAY` in
`Constants.java` and run the simulation; it asks for a log file and writes a new log ending in
`_sim`. Open both in AdvantageScope and compare, e.g. to see how different vision std devs would
have changed the pose estimate on real data. See the AdvantageKit docs for details.
