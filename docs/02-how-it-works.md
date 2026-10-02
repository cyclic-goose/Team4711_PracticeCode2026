# 02: How it works

This walks through the ideas in the order they build on each other, with pointers into the code.
Read it alongside the files.

---

## 1. Field coordinates

Everything uses the WPILib **"always blue origin"** convention:

- `(0, 0)` is the corner of the field at the **blue** alliance wall, on the right side when you
  stand behind the blue driver station.
- **+X** points from the blue wall toward the red wall (16.54 m long).
- **+Y** points to the left as seen from the blue driver station (8.07 m wide).
- **Heading 0°** means facing +X. Angles grow **counter-clockwise** (turning left is positive).

Red positions are never written by hand. You write the blue version and **flip** it. The 2026
field is *rotationally* symmetric: a red spot is the blue spot rotated 180° around the center of
the field. PathPlanner flips this way by default, and a unit test (`FieldConstantsTest`) checks
that flipping the blue hub lands exactly on the red hub.

The **hub centers** aren't typed in. `FieldConstants` computes them from the official 2026 tag
map, from the box the 8 tags on each hub outline.

## 2. Odometry: where am I, from the wheels?

Swerve **odometry** adds up how far each wheel rolled and which way it pointed, together with the
gyro (Pigeon 2) heading, to track the robot's position. It updates 100 times a second and is
smooth, but it **drifts**: wheels slip, carpet compresses, and small errors add up. After a
match's worth of driving it can be off by a foot or more.

Code: `Drive.periodic()`, which runs `poseEstimator.updateWithTime(...)` for each odometry sample.

The heading must start out right. Odometry only knows *changes* in heading, so it has to be told
which way the robot faces at the start. That's what the **Start button** does ("I'm facing away
from my driver station"). In autonomous, PathPlanner sets the starting pose from the auto.

## 3. Vision pose: where am I, from the tags?

The Limelight finds tags in the image. It knows where every tag is on the field (the field map)
and where the camera is on the robot (the robot-space camera pose), so it can work out where the
**robot** is on the field. It does this two ways:

- **MegaTag 1** (`botpose_wpiblue`): solves the full position and heading from the tag corners
  alone. With one tag, the solution can be **ambiguous**: two different poses produce nearly the
  same image, so it sometimes flips between them. With two or more tags it's solid.
- **MegaTag 2** (`botpose_orb_wpiblue`): the robot code sends its gyro heading to the Limelight
  every loop (`robot_orientation_set`). Since heading is known, there's nothing ambiguous left, and
  even **one tag** gives a stable position. The catch: if the robot's heading is wrong, the
  position is wrong. Another reason the Start button matters.

Code: `VisionIOLimelight.updateInputs()` reads both and sends the heading.

## 4. Pose estimation: combining both

Wheels are smooth but drift. Vision doesn't drift but is noisy and only works when a tag is
visible. WPILib's `SwerveDrivePoseEstimator` blends them: every vision measurement pulls the
estimate toward it, weighted by how much each source is trusted.

That trust is set with **standard deviations**, in meters and radians. A small std dev means
"trust this a lot", a large one means "trust it a little". `Vision.periodic()` sets them per
measurement:

```
stdDev = baseline × distance² / numberOfTags
```

So a close tag counts for much more than a far one, and two tags more than one. MegaTag 2 gets
an infinite heading std dev ("don't use my heading"), because its heading came from the gyro in
the first place.

Bad measurements are thrown out entirely: no tags, an ambiguous single tag, impossible height,
or off the field. Accepted and rejected poses are both logged so you can see them in AdvantageScope
(`Vision/Camera0/RobotPosesAccepted` / `Rejected`).

**Latency:** each camera frame is stamped with the moment the image was *captured*, not when
the code got it (30–60 ms later). The estimator applies the correction at that earlier moment in
its history and replays forward. That's why vision doesn't make the pose jitter.

## 5. Aiming

### Why the old code was slow and wobbly

The old `centerOnTag` did `omega = PID(tx)`: turn speed straight from where the tag appeared in
the image. Two problems:

1. **A units bug.** Its motion profile was limited to "8 per second", which was meant as
   8 *radians*/s for the gyro, but the input was `tx` in *degrees*. So it could only turn about
   8°/s regardless of kP. That's why it was slow.
2. **Steering by a slow, late sensor.** The camera delivers a new `tx` 20–50 times a second, each
   30–60 ms old, and sometimes not at all (dropped frames). Turn up kP and the robot reacts to
   where the tag *was*, overshoots, sees it on the other side, overcorrects... that's the
   oscillation. When the tag dropped, rotation went to zero and the controller wasn't reset, so
   the next frame caused a jump.

### Level 1: aim with tx (`AimController.aimWithLimelightTx`, the **Y** button)

Use the camera only to decide **which heading to turn to**, and the **gyro** to get there:

```
when a NEW frame arrives with a tag:
    headingWhenCaptured = drive.getRotationAt(frame.timestamp)   // undo the latency
    targetHeading = headingWhenCaptured - tx                      // tx is positive to the right
every loop (50/s):
    omega = PID(targetHeading - currentGyroHeading)
```

The gyro is fast, accurate and never drops out, so the control is smooth. A dropped frame just
means the target heading isn't updated that cycle. The simulated test
`level1TxAimWorksDespiteLatencyAndDroppedFrames` uses a fake camera that **drops 50% of frames**
with 60 ms latency, and the aim still settles within 1.5°.

Level 1 needs no field map and no camera position, just a visible tag. It's the quickest win on
the real robot.

### Level 2: aim at a field position (`AimController.aimAtHub`, the **A** button)

Once the robot knows where it is on the field (section 4), aiming needs no camera at all:

```
targetHeading = direction from robot to hub center  (minus SHOOTER_FACING)
omega = feedforward + PID(targetHeading - currentHeading)
```

Benefits:

- It keeps aiming **with no tag in view**; odometry carries it between tag sightings.
- It knows the **distance**, which sets the shooter speed.
- It handles **driving while aiming**. As you strafe past the hub, the direction to the hub sweeps
  around. The **feedforward** (`AimingMath.bearingRateRadPerSec`) predicts that sweep from your
  joystick velocity, so the robot turns *with* it instead of lagging behind. Tested by
  `level2AimStaysOnTargetWhileStrafing`.

The cost: the camera pose must be set correctly in the Limelight, and the heading must be right
(Start button / auto starting pose).

### The heading controller (PID + feedforward)

Both levels share one controller with these live-tunable gains under `/Tuning/Aim/`:

- **kP**: turn speed per unit of error. More = snappier, too much = overshoot and wobble.
- **kD**: damping. Resists fast changes in error, reducing overshoot. Too much = sluggish/twitchy.
- **MaxOmegaRadPerSec**: speed limit on turning.
- **ToleranceDeg**: how close counts as "aimed" (feeding and rumble wait for this).

How to tune on the robot: set kD = 0. Raise kP until it snaps to the target and *just* starts to
overshoot. Back kP off a little, then add a small kD to clean up any remaining overshoot. Watch
`Aim/ErrorDegrees` in AdvantageScope while you do it.

## 6. Shooter speed from distance

**Velocity control:** the flywheel runs `VelocityVoltage` on the TalonFX: "hold 3000 RPM". The
TalonFX runs the control loop itself 1000 times a second:

```
volts = kS·sign(target) + kV·target + kP·(target − measured)
```

- **kV** does most of the work: the volts needed per unit of speed. Estimate it as
  12 V ÷ free speed.
- **kS** overcomes friction.
- **kP** corrects what's left over, like the dip when a ball goes through.

This is far more consistent than percent output, which changes with battery voltage.

**The table:** `ShooterConstants.DISTANCE_TO_RPM` maps distance (m) to RPM. In between entries it
**interpolates** in a straight line. You build it on the robot: park at a distance, adjust
`/Tuning/Shooter/ManualRPM` (D-pad up) until shots go in, write down `Field/DistanceToHubMeters`
and the RPM, and repeat at 4–6 distances.

## 7. PathPlanner

**PathPlanner** has two parts: a desktop **GUI** where you draw paths and autos, and a
**library** (PathPlannerLib) in the robot code that follows them.

- **Paths** (`deploy/pathplanner/paths/*.path`): curves with a start and end heading and speed
  limits. Always drawn for **blue**; flipped automatically on red.
- **Autos** (`deploy/pathplanner/autos/*.auto`): sequences of paths and **named commands**, e.g.
  "follow path, then Shoot". Named commands are registered in
  `RobotContainer.registerNamedCommands()`, and **must be registered before** the auto chooser is
  built.
- **AutoBuilder** (configured in the `Drive` constructor) tells PathPlanner how to read the pose,
  reset it, read speeds, and drive. It's also where the path-following PID lives
  (`PPHolonomicDriveController`, translation and rotation kP = 5).
- **resetOdom: true** in an auto sets the robot's pose to the path's start at the beginning of
  auto. The robot has to actually be placed there.
- **Robot config** (`Drive.PP_CONFIG` + `settings.json`): mass, MOI, wheel size, gearing, motor
  type. PathPlanner uses these to work out how fast the robot can safely accelerate. Wrong values
  mean paths it can't actually follow.

**Pathfinding** (`AutoBuilder.pathfindToPoseFlipped`, the **D-pad down** button) builds a path on
the fly from wherever the robot is to a target, around obstacles in `navgrid.json`. The included
navgrid marks the field walls and both hubs. Each hub has 0.65 m of padding: half the bumper width
plus room for the path smoothing, which cuts corners slightly. **Not marked:** the bumps, trenches
and towers; I wasn't certain enough of their exact sizes. Open the navgrid editor in the
PathPlanner GUI and paint them before pathfinding near them. The simulated test
`pathfindingDrivesAroundTheHubToTheShootingSpot` checks the robot goes around the hub, not through
it.

**When a path misses its end point:** first check the pose is right (vision + Start button),
then the robot config, then the path-following PID.

## 8. AdvantageKit: the IO layer, logging and replay

Every subsystem is split in two:

- The **subsystem** (`Shooter.java`) holds the logic and only talks to an interface (`ShooterIO`).
- **IO implementations** do the hardware: `ShooterIOTalonFX` (real), `ShooterIOSim` (physics),
  or an empty one (replay).

All sensor values pass through an `@AutoLog` "inputs" object, which AdvantageKit records every
loop. That gives you:

1. **Simulation**: the same subsystem code runs against physics models on your laptop.
2. **Full logs**: every sensor value, every output, every loop, saved to the USB stick.
3. **Replay**: feed a real log back through *changed* code and see what the new code would have
   done with the same sensor data. Useful for tuning vision filtering without the robot.

**TunableNumber** (`util/TunableNumber.java`): any value under `/Tuning/` can be changed live, and
changes are logged. Copy good values back into the code, because dashboard values don't save.

## 9. Where to go next

Once Levels 1 and 2, the shooter table and PathPlanner autos all work:

- **Shoot on the move:** aim at a "virtual hub" offset by robot velocity × ball flight time.
- **Better pose trust:** tune `VisionConstants` std devs from logs; reject MegaTag2 while
  spinning very fast.
- **Limelight 4 internal IMU:** `imumode_set` modes let the camera use its own gyro. The Pigeon 2
  is excellent, so this isn't needed yet.
- **A more realistic sim:** maple-sim gives a "true" robot pose separate from odometry, which
  makes vision testing in sim meaningful (see doc 03).
