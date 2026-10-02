# Flying Aces 4711 — Limelight + Path Planning Trials (Oct 2026)

A fresh robot project built to learn vision (Limelight 4) and path planning (PathPlanner)
properly before the 2027 season, using the 2026 robot.

It is the official **AdvantageKit TalonFX swerve template** + AdvantageKit's **vision template**,
with your `TunerConstants` and intake moved over, plus:

- **Fixed tag aiming** (Level 1): the old "center on tag" rebuilt so it's fast and doesn't care
  about dropped frames.
- **Field-position aiming** (Level 2): aims at the hub using where the robot *is* on the field,
  so it keeps aiming even when no tag is visible.
- **Limelight odometry**: MegaTag poses fused into the swerve pose estimator, with filtering and
  logging so you can see how good it is.
- **PathPlanner autos** (classroom-sized examples that show each feature) and **drive-to-a-spot**
  (on-the-fly pathfinding).
- **Logging + simulation + tests**, so most of the work happens on a laptop and robot time is
  for verifying.

## Start here (in this order)

1. **[docs/01-limelight-bench-test.md](docs/01-limelight-bench-test.md)**: fix the tag detection
   problem with the Limelight on a desk. Nothing else works well until detection is solid.
2. **[docs/02-how-it-works.md](docs/02-how-it-works.md)**: the concepts (odometry, MegaTag,
   pose estimation, aiming, PathPlanner) and where each one lives in the code.
3. **[docs/03-simulation-and-logs.md](docs/03-simulation-and-logs.md)**: run the robot on your
   laptop, view it in AdvantageScope, and review logs from robot sessions.
4. **[docs/04-robot-session-plans.md](docs/04-robot-session-plans.md)**: checklists for each
   Tue/Thu 2-hour session, in order.
5. **[docs/05-pathplanner-guide.md](docs/05-pathplanner-guide.md)**: how PathPlanner works from
   start to finish, plus a tutorial for making and running your own paths and autos.

## Quick start

All from a terminal in this folder (or use the WPILib commands in VS Code):

```powershell
./gradlew build      # compile + run all tests
./gradlew test       # just the tests (includes simulated-robot tests)
./gradlew simulateJava   # run the robot program in simulation (no GUI; see docs/03 for the GUI)
./gradlew deploy     # send to the robot (connected to the robot's network)
```

If `gradlew` can't find Java, use WPILib VS Code (it sets this up for you), or first run
`$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"`.

## Controller map (Xbox controller, port 0)

| Input | Action |
| --- | --- |
| Left stick | Drive (field-relative, **50% speed by default**, see below) |
| Right stick X | Rotate |
| **Start** | "The robot is facing away from me": sets heading (0° blue / 180° red). Press whenever you place the robot. |
| **A** (hold) | **Level 2 aim**: face our hub using the field pose (works with no tag in view, and shows whether the pose is right) |
| **Y** (hold) | **Level 1 aim**: turn to center whatever tag the Limelight sees |
| X | Lock wheels in an X |
| B | Toggle intake lock (retract + ignore intake controls), same as before |
| LB / RB | Intake deploy / retract, same as before |
| RT / LT | Intake rollers in / out, same as before |
| D-pad down (hold) | Drive itself to the aiming spot in front of the hub (PathPlanner pathfinding), then aim. Let go to stop. |
| D-pad up (hold) | Drive itself back to the start of the Classroom autos (so you don't push it back by hand) |
| **Back** | Save a Limelight snapshot + the last 20 s of video (Limelight 4 Rewind) |
| Rumble | Aimed (during A, or the AimAtHub auto command) |

**Speed limit:** teleop starts at 50% speed for classroom safety. Change it live at
`/Tuning/Drive/TeleopSpeedScale` (1.0 = full speed). Everything under `/Tuning/` can be changed
live from AdvantageScope or Elastic without redeploying; copy good values back into the code.

## Set these before driving the real robot

Search the code for `TODO` (VS Code: Ctrl+Shift+F). The important ones:

| What | Where |
| --- | --- |
| Which side should face the hub when aiming (front or back) | `AimController.AIM_SIDE` (front by default) |
| Camera position on the robot | **Limelight web UI** (robot-space camera pose) **and** `VisionConstants.robotToCamera0` (for sim) |
| Robot mass (with bumpers + battery) | `Drive.ROBOT_MASS_KG` and `pathplanner/settings.json` |
| Bumper size | `pathplanner/settings.json` (`robotWidth`, `robotLength`) |

The Limelight must be named `limelight` (the default) and set to a static IP of `10.47.11.11`.

## Project layout

```
src/main/java/frc/robot/
  Robot.java, RobotContainer.java   setup + controller bindings + auto chooser
  Constants.java                    sim/real/replay mode, CAN IDs, tuning-mode switch
  FieldConstants.java               2026 field, hub centers (computed from the tag map), flipping
  commands/
    AimController.java              Level 1 (tx) and Level 2 (field pose) aiming
    DriveCommands.java              joystick driving + drivetrain characterization
  subsystems/
    drive/                          swerve (AdvantageKit template), PathPlanner setup
    vision/                         Limelight + simulated camera, pose filtering, detection stats
    intake/                         same behavior as the old intake
  util/
    AimingMath.java                 the aiming math (unit tested)
    TunableNumber.java              numbers you can change live from the dashboard
src/main/deploy/pathplanner/        paths, autos, navgrid, robot settings (edit with PathPlanner GUI)
src/test/java/frc/robot/            unit tests + simulated-robot tests; AllPathsAndAutosTest checks
                                    every path and auto you make
tools/Test-LimelightDetection.ps1   measures Limelight detection rate from a laptop
docs/                               the guides listed above
```

## What changed from the old code

- **The slow centering was a units bug.** The old `centerOnTag` profiled controller reused
  `ANGLE_MAX_VELOCITY = 8` / `ANGLE_MAX_ACCELERATION = 20`, which were meant as radians for the
  gyro controller, but its input was `tx` in **degrees**, so it could never turn faster than
  about 8°/s, whatever kP was. Raising kP then made it oscillate, because the camera signal arrives
  late and less often than the robot loop runs. Level 1 aiming fixes both problems; see
  docs/02.
- **Field-relative driving is the template's standard version.** The old code flipped it
  (`isFlipped = !Red`) for normal driving but not for the A-button lock, so the two disagreed.
  Now it's consistent, and **Start** sets the heading correctly for your alliance.
- **The time-based autos are gone.** Replaced by PathPlanner (see the `Classroom` autos).
  The old code is still on the `PostSeasonCelebrationSpeed` branch on GitHub.
- **No shooter.** It's broken on this robot, so the shooter code was removed. It's in this
  repo's git history if you want it back.
- **Logging is back on (AdvantageKit).** Put a USB stick in the roboRIO and every session gets
  recorded.
