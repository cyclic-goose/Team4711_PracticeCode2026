# 05: PathPlanner — how it works and how to use it

Part A explains what happens inside, from a drawing in the GUI to wheels turning. Part B is a
step-by-step tutorial for making and running your own paths and autos. Part C covers fixing
problems.

---

# Part A: How it works

## A1. The pipeline

```
 YOU (PathPlanner GUI)           ROBOT CODE (PathPlannerLib, every 20 ms)
 ─────────────────────           ─────────────────────────────────────────────────────────
  Path file (.path)  ──────►  1. TRAJECTORY: turn the curve into a timed plan:
  a curve + rules              "at t = 1.30 s be at (2.1, 4.6), facing 35°, moving 1.2 m/s"
                                          │
                               2. FOLLOWER: every loop, look up where the plan says the robot
                                  should be right now, compare with where it IS (the pose),
                                  and command a velocity:
                                      speed = planned speed  +  kP × (position error)
                                          │
                               3. DRIVE: Drive.runVelocity() turns that into 4 wheel speeds
                                  and angles
                                          │
                               4. POSE: odometry (wheels + gyro) + Limelight corrections
                                  answer "where am I?" ──────► back to step 2
```

Two things to take from this:

1. **Most of the motion is planned ahead.** The trajectory already contains the speed for every
   moment. The PID part only corrects small drift. That's why paths look smooth.
2. **The follower is only as good as the pose.** If the robot *thinks* it's 30 cm to the left of
   where it really is, it will follow the path perfectly... 30 cm off. Good Limelight odometry
   and path planning are really one problem.

## A2. Paths: what you draw

A **path** (`src/main/deploy/pathplanner/paths/*.path`) is a smooth curve plus rules for driving
it.

### Waypoints and control points

The curve is a chain of **Bézier curves**. Each **waypoint** has an anchor (a point the robot
passes through) and up to two **control points** (handles). Handles set the direction the robot
travels through the anchor, and how long the curve "keeps going straight" before it bends. Longer
handles give wider, gentler curves.

Positions along a path are measured in **waypoints**: 0.0 is the first waypoint, 1.0 the second,
1.5 halfway between the second and third, and so on. Rotation targets, zones and markers all use
this.

### Rotation is separate from travel direction (holonomic)

A swerve robot can drive one way while facing another. A path sets heading with:

- **Ideal starting state:** the heading (and speed) at the start.
- **Goal end state:** the heading (and speed) at the end. End speed 0 means stop.
- **Rotation targets:** "be facing X° when you reach position P". PathPlanner turns smoothly
  between targets.
- **Point-towards zones:** "between positions A and B, keep facing this spot on the field". Great
  for keeping the camera or a mechanism pointed at a target while moving.

### Speed limits

- **Global constraints:** max speed, acceleration, turn rate and turn acceleration for the path.
- **Constraint zones:** different (usually slower) limits between two positions, e.g. slow down
  near a wall.
- The **robot config** adds physical limits: how much force the wheels can push without slipping,
  given the motors, gearing, current limit and wheel grip. PathPlanner never plans more
  acceleration than the robot can produce.

### Event markers

An **event marker** runs a command at a point (or across a zone) along the path while the robot
keeps driving: "deploy the intake at position 1.5". The command is a **named command** (A5).

### Linked waypoints

Give waypoints in different paths the same **linked name** and moving one moves all of them. Use
this where one path ends and the next begins, so they always line up.

### Coordinates and red/blue

Paths are always drawn on the **blue** side (blue-origin coordinates, see doc 02). On red,
PathPlanner rotates the whole path 180° around the field center, because the 2026 field is
rotationally symmetric. That includes rotations and point-towards spots. You never draw red paths.
(Controlled by the last-but-one argument to `AutoBuilder.configure` in `Drive.java`: "flip if red".)

### The example paths in this project

They're all sized to fit in a classroom in front of a taped-up hub face (see A8):

| Path | What it shows |
| --- | --- |
| `Example - Back Up` | The simplest path: two waypoints, straight line, no rotation |
| `Classroom - Loop` | 5 waypoints making a loop; a **rotation target** (face 150° at the far side and back again); an **event marker** "Halfway" that runs the `LogMarker` named command |
| `Classroom - Orbit Out` | An arc around the hub with a **point-towards zone** (always faces the hub, so it strafes sideways) and a **constraint zone** ("Slow Zone", 0.5 m/s in the middle) |
| `Classroom - Orbit Back` | Comes back along the arc; its first waypoint is **linked** ("Orbit End") to the end of Orbit Out |

## A3. Trajectory generation: from curve to timed plan

When a path starts (`AutoBuilder.followPath(path)` or a path inside an auto), PathPlannerLib turns
it into a **trajectory**: a list of states, each with a time, position, heading, velocity and
turn rate.

Roughly:

1. Sample the curve into many closely spaced points.
2. At each point, find the fastest safe speed: the speed limit (global or zone), slowed down for
   tight curves (sideways acceleration), and limited by what the motors and wheel grip allow.
3. Go forward along the points, speeding up only as fast as the robot can accelerate. Then go
   backward, so the robot starts braking early enough to hit each slow section and the end
   speed.
4. Fill in heading over time from the start state, rotation targets, point-towards zones and end
   state.
5. Add up the time between points. This gives the total time, and the "where should I be at time
   t" table.

It starts from the robot's **current** speed and heading, so a path can begin while moving.

This is why the **robot config** matters (`Drive.PP_CONFIG` in code, `settings.json` for the
GUI): mass, rotational inertia (MOI), wheel radius, gearing, motor type (Kraken X60), current
limit, top speed, wheel grip (COF) and module positions. **The code's config is what the robot
uses.** The GUI's copy is for previews, and PathPlannerLib shows an alert if the two disagree.
Both currently match.

## A4. Following: the PPHolonomicDriveController

Every 20 ms while a path runs:

```
target = trajectory.sample(timeSinceStart)        // where the plan says to be
error  = target.pose − drive.getPose()            // where we are vs the plan
vx, vy = target.velocity  +  kP_translation × position error
omega  = target.turnRate  +  kP_rotation    × heading error
drive.runVelocity(robot-relative version of vx, vy, omega)
```

The gains are `PATH_TRANSLATION_PID` and `PATH_ROTATION_PID` in `Drive.java` (kP = 5 for both,
the template defaults). Logged for tuning:

| Log key | Meaning |
| --- | --- |
| `Odometry/Trajectory` | The whole path being followed (draw it on the 3D field) |
| `Odometry/TrajectorySetpoint` | Where the robot should be right now (show as a ghost robot) |
| `Odometry/Robot` | Where the robot thinks it is |
| `PathPlanner/TranslationErrorMeters` | Distance between the two. Should stay under ~5–10 cm. |
| `PathPlanner/RotationErrorDegrees` | Heading difference. Should stay under a few degrees. |

The path command ends when the trajectory's time runs out. If the robot is still a bit off at
that moment, it stops there; PathPlanner doesn't keep correcting after the end.

## A5. Autos and named commands

An **auto** (`autos/*.auto`) is a tree of command blocks:

| Block | Meaning |
| --- | --- |
| **Path** | Follow one path |
| **Named Command** | Run a command the code registered under a name |
| **Wait** | Pause for some seconds |
| **Sequential group** | Run children one after another |
| **Parallel group** | Run children together; finish when **all** finish |
| **Race group** | Run together; finish when **any one** finishes |
| **Deadline group** | Run together; finish when the **first** child finishes |

`Classroom - Tour` is: Loop → Orbit Out → wait 0.5 s → Orbit Back → `AimAtHub`.

**Named commands** connect the GUI to your Java code. In `RobotContainer.registerNamedCommands()`:

```java
NamedCommands.registerCommand("AimAtHub", aim.aimAtHub(() -> 0.0, () -> 0.0).until(aim::isAimed).withTimeout(2.0));
NamedCommands.registerCommand("LogMarker", Commands.runOnce(() -> Logger.recordOutput("Auto/LastMarkerTime", Timer.getTimestamp())));
```

The name in the GUI must match **exactly** (capitals, spaces). If it doesn't, PathPlanner only
prints a warning and skips that block, so the robot just doesn't do it. `AllPathsAndAutosTest`
catches this for you. Named commands must be registered **before**
`AutoBuilder.buildAutoChooser()` runs; they are in this code.

**resetOdom** (on by default in an auto): when the auto starts, set the robot's pose to the first
path's starting pose (flipped for red). That only helps if the robot is actually placed there.
With good Limelight odometry, the vision corrections pull the pose to the truth within a second
either way.

**Running an auto:** every `.auto` file appears in the `Auto Choices` chooser
(`AutoBuilder.buildAutoChooser()`). Pick it on the dashboard and enable Autonomous.

## A6. Pathfinding: paths made on the fly

`AutoBuilder.pathfindToPose(target, constraints)` (D-pad down / D-pad up) builds a path from
wherever the robot is right now:

1. The field is divided into 0.3 m squares, from `navgrid.json`. Blocked squares are the walls
   and the hubs (with 0.65 m padding for the bumpers, and because smoothing cuts corners).
2. A search algorithm (AD\*, a relative of A\*) finds a route through free squares. It runs on a
   background thread.
3. The route is simplified to a few waypoints and smoothed into a normal path.
4. That path is followed exactly like a drawn one (A3 + A4).

Useful variants:

- `pathfindToPoseFlipped(bluePose, ...)`: you give a blue pose; it's flipped on red.
- `pathfindThenFollowPath(path, ...)`: pathfind to the **start** of a hand-drawn path, then
  follow that path. Best for precise endings: pathfinding gets you close, your drawn path gives a
  repeatable final approach.

The included navgrid does **not** include the bumps, trenches or towers. Paint them in the GUI's
navigation grid editor before pathfinding near them.

## A7. Where the Limelight comes in

The follower trusts `drive.getPose()` completely, so vision quality shows up directly as
path-following quality:

- **A good pose** means the robot ends where the path says, every time, even if it was placed a
  little off.
- **A vision jump mid-path** (a bad measurement accepted) means the pose suddenly moves, the
  error jumps, and the robot jerks to "correct". Look for spikes in
  `PathPlanner/TranslationErrorMeters` and check `Vision/Camera0/RobotPosesAccepted` at that
  moment.
- **No tags visible** means wheel odometry carries it. That's fine for a few seconds, then it
  slowly drifts.
- **A wrong camera pose in the Limelight** means every vision measurement is off by the same
  amount, so the robot repeatably ends in the wrong spot.

After any pose reset (auto start, Start button), `Drive.addVisionMeasurement` ignores camera
frames captured *before* the reset. Otherwise those old frames would yank the pose right at the
start of the auto.

## A8. The classroom

You'll practice with the **red hub face** taped on the wall (tags 10 and 9, see doc 01) and the
Driver Station on **Red**. You draw paths on the **blue** side; red flips them onto the same spot
in front of your wall.

So in the GUI, think of the blue hub's front face (x = 4.02 m, between y = 3.43 and 4.64) as
**your wall**. Keep paths in the area in front of it:

```
  y (m)
  5.2 ┤  ┌────────────────────────┐      │
      │  │  keep the robot's      │      │
  4.4 ┤  │  CENTER in this box    │      █  tag 25 ┐ hub face
 4.03 ┤  │                  S ──► │      █  tag 26 ┘ (= your wall)
  3.6 ┤  │                        │      │
  2.9 ┤  └────────────────────────┘      │  wall at x = 4.02
      └──┬───────────┬────────────┬──────┴──► x (m)
        1.0         2.0          3.3    4.02
```

`S` is the classroom start, with the robot facing the wall (arrow).

- Robot **center** at x ≤ ~3.3 m (bumpers 0.45 m + some margin from the wall).
- Classroom start `S` = (2.6, 4.03), facing 0°: centered on tag 10, 1.42 m (56 in) from the wall to
  the robot center, front facing the wall.
- Watch the GUI's preview animation (the robot outline) to check nothing swings into a desk.

---

# Part B: Tutorial — your own path and auto

## B1. Install the GUI

Install **PathPlanner** from the Microsoft Store, or from the GitHub releases page
(`mjansen4857/pathplanner`). Use a **2026** version to match the library in this project
(PathplannerLib 2026.1.2, in `vendordeps/PathplannerLib.json`).

## B2. Open this project

Open the project folder: the one with `build.gradle` in it. PathPlanner finds
`src/main/deploy/pathplanner` by itself. You'll see the `Classroom` folders of paths and autos.

Open **Settings** and look at the robot config. It should already match the code (Kraken X60,
54 kg, module positions ±0.2667 m...). **If you change a value here, change it in
`Drive.PP_CONFIG` too.**

## B3. Draw a path

1. **Paths** → **+** (new path). Name it, e.g. `My First Path`.
2. Drag the start and end waypoints onto the field (blue side, inside the classroom box, A8).
   Click the path to add a waypoint in between; drag the handles to shape it.
3. In the side panel:
   - **Ideal Starting State** rotation: which way the robot faces at the start (0° = facing the
     wall in the classroom).
   - **Goal End State**: end rotation, and velocity 0.
   - **Global Constraints**: start with **1.0 m/s** and **1.0 m/s²** for the classroom.
4. Optional: add a **Rotation Target**, a **Point Towards Zone** (pick the hub center,
   (4.63, 4.03)), a **Constraint Zone**, or an **Event Marker** with the `LogMarker` command.
5. Press the preview/play control to watch the robot outline drive it. Check it stays clear of
   the wall.

PathPlanner saves as you go, straight into `src/main/deploy/pathplanner/paths/`.

## B4. Make an auto

1. **Autos** → **+**. Name it, e.g. `My First Auto`.
2. Add a **Path** block → `My First Path`.
3. Add a **Named Command** block → `AimAtHub`.
4. Leave **Reset Odometry** on.

## B5. Check it on your laptop

```powershell
./gradlew test
```

`AllPathsAndAutosTest` automatically:

- drives **your new path** in simulation and checks it ends within 15 cm and 5° of where it should;
- checks **your new auto** only uses paths that exist and named commands that are registered.

The test report is at `build/reports/tests/test/index.html`. Your path shows up as
`path: My First Path`.

## B6. Watch it in simulation

1. VS Code → **WPILib: Simulate Robot Code** → tick **Sim GUI**.
2. Sim GUI: alliance **Blue 1** (or Red 1, to check the flip).
3. AdvantageScope → Connect to Simulator → 3D Field (2026). Add `Odometry/Robot` (robot),
   `Odometry/TrajectorySetpoint` (ghost) and `Odometry/Trajectory` (trajectory).
4. Pick `My First Auto` in `Auto Choices` (Elastic, or the Sim GUI's NetworkTables view).
5. Sim GUI → **Autonomous**. Watch it drive.

## B7. Run it on the robot

1. **Deploy** (`./gradlew deploy`). Paths live in the deploy folder on the roboRIO, so **every
   time you change a path, deploy again.**
2. Driver Station on **Red** (tags 10 and 9 on the wall). Check the pose looks right in
   AdvantageScope before starting.
3. Place the robot at the auto's start (the classroom autos: centered on tag 10, 56 in from the
   wall to the robot center, facing the wall).
4. Pick the auto, clear the area, hand near **Enter** (disable) / **Space** (e-stop), enable
   Autonomous.
5. Afterwards, look at `PathPlanner/TranslationErrorMeters` in the log.
6. Between runs, **hold D-pad up** and the robot drives itself back to the classroom start.

## B8. Tuning path following on the robot

Do it in this order. Each step depends on the one before.

1. **Pose first.** Robot parked: is `Odometry/Robot` where it really is (tape measure)? If not,
   fix vision/camera pose (doc 01) before anything else.
2. **Wheel radius.** Run "Drive Wheel Radius Characterization" (doc 04). If it's wrong, odometry
   distances are wrong.
3. **Run `Classroom - Loop`** and watch `TranslationErrorMeters`:
   - **Small (< ~5 cm) the whole way:** done.
   - **Grows steadily and the robot lags behind the ghost:** the plan asks for more than the robot
     can do. Lower the path's speed/acceleration, check the mass and wheel COF in the robot
     config, then raise `PATH_TRANSLATION_PID` kP a little.
   - **Robot wiggles side to side around the path:** kP too high. Lower it (try 3).
   - **Sudden spikes:** vision jumps (A7).
4. **Rotation** the same way, with `RotationErrorDegrees` and `PATH_ROTATION_PID`.

---

# Part C: Troubleshooting

| Symptom | Likely cause → fix |
| --- | --- |
| Auto doesn't show in the chooser | File isn't in `deploy/pathplanner/autos`, or you didn't redeploy |
| Robot runs the **old** version of a path | Redeploy; paths are files on the roboRIO |
| Robot drives the mirror image / to the wrong half | Driver Station alliance is wrong (Red vs Blue) |
| Robot confidently goes to the wrong spot | The pose is wrong: placement, heading (Start button), or Limelight camera pose |
| Robot lags behind the whole path | Path too fast for the real robot, wrong robot config, or wrong wheel radius |
| Robot oscillates around the path | Path PID too high, or vision measurements jumping |
| A named command block "does nothing" | Name not registered / typo. `./gradlew test` tells you which. |
| An event marker never fires | Same as above; also check its position is within the path's length |
| Pathfinding drives through something | That obstacle isn't painted in the navgrid |
| Pathfinding does nothing | The target is inside a blocked navgrid square, or there's no free route |
| Alert about config mismatch | GUI settings ≠ `Drive.PP_CONFIG`. Make them match. |

## Glossary

- **Pose:** position (x, y) and heading on the field.
- **Odometry:** tracking the pose from wheel movement + gyro.
- **Trajectory:** a path with timing: where to be at every moment.
- **Feedforward:** the planned speed, sent without waiting for an error.
- **Feedback (PID):** correction based on error between plan and pose.
- **Holonomic:** can move in any direction while facing any direction (swerve).
- **Navgrid:** the map of blocked squares used by pathfinding.
- **Named command:** a Java command registered by name so the GUI can use it.
