# 04: Robot session plans (Tue/Thu, 2 hours)

Each session has **one main goal**, a short checklist, and a "done when". If a session finishes
early, start the next one. If it doesn't, repeat it. Between sessions: put the measurements and
tuned values into the code, run `./gradlew test`, and try changes in sim.

## Every session

- [ ] **Before leaving home:** `./gradlew build` passes; commit to git so you can roll back.
- [ ] **USB stick** in the roboRIO (that's where the logs go).
- [ ] AdvantageScope layout ready; laptop charged.
- [ ] **Safety:** teleop starts at **50% speed** (`/Tuning/Drive/TeleopSpeedScale`). On the
      Driver Station, **Enter = disable** and **Space = emergency stop**. Keep a hand near them.
      Clear the floor before autos or pathfinding.
- [ ] **After:** download the log (AdvantageScope → File → Download Logs) and write a few notes on
      what changed and what you saw.

The old code is still on the `PostSeasonCelebrationSpeed` branch on GitHub. If something goes
badly wrong, you can redeploy that.

---

## Before session 1 (at home)

- [ ] **Bench test the Limelight** ([01](01-limelight-bench-test.md)). Bring it home next class.
      Print proper tags (10, 9, 26, 25).
- [ ] Run the simulation and drive around in it ([03](03-simulation-and-logs.md)). Try every
      button so you know what to expect.

## Session 1: new code running, take measurements

1. **Robot on blocks.** Deploy. The drivetrain uses your existing `TunerConstants`, so the
   modules should behave exactly as before. Check each wheel drives and steers.
2. **Measure** (write everything down):
   - Camera: forward / sideways / up from the **robot center at floor level**, and its tilt.
   - Does the shooter shoot out the **front** or the **back**?
   - Robot weight with bumpers and battery; bumper outer length and width.
   - Drive motors: Kraken X60 or Falcon 500?
   - Flywheel gear ratio (motor turns per flywheel turn).
3. **Floor, 50% speed:** point the robot away from you, press **Start**, and drive. Forward on the
   stick should move away from you whatever way the robot is turned.
4. **Intake:** bumpers, triggers and B lock should work exactly like before.
5. **Wheel radius:** pick **Drive Wheel Radius Characterization** in the auto chooser, enable
   Autonomous, and let the robot spin slowly in place for 2–3 turns, then disable. The result
   prints in the Driver Station console. Put it in `TunerConstants.kWheelRadius`.
6. **Limelight:** apply your bench-test settings. Set IP `10.47.11.11`, name `limelight`, and the
   2026 field map. Enter the camera pose. If you can, move power off the VRM 500 mA output (see
   doc 01).
7. **Tape up tags 10 and 9** on a wall (doc 01, "Testing the field-position features in a
   classroom"). Leave them there if you're allowed to.

**Done when:** the robot drives field-relative, the intake works, and you have the measurements.

**At home:** fill in the `TODO`s (`SHOOTER_FACING`, `robotToCamera0`, mass, motor type, gear
ratio, bumper size) and run the tests.

## Session 2: vision and Level 1 aim (Y)

1. With the Driver Station on **Red**, park the robot facing the tags at 3, 6 and 10 ft. Check
   `Vision/Camera0/DetectionRatePercent`. It should be near 100%.
2. **Pose check:** park at a measured spot. Tag 10 is at field position (12.52, 4.03), facing +X,
   so 2 m straight out from tag 10 is (14.52, 4.03). Compare with `Odometry/Robot`. A few inches
   off is fine.
3. **Level 1 aim (hold Y):**
   - Robot still, pointed 30° away from the tags → it should snap to them.
   - Hold Y while strafing slowly → it should keep facing the tag.
   - Tune `/Tuning/Aim/kP` and `kD` (doc 02, "The heading controller"). Watch
     `Aim/ErrorDegrees`.
4. **Any time the tag drops, press Back** (snapshot + 20 s video).

**Done when:** detection holds to about 12 ft and Y snaps to the tag in under ~0.5 s without
wobbling.

## Session 3: Level 2 aim (A) and pose quality

1. Press **Start** facing away from the wall, drive around, and watch the 3D field in
   AdvantageScope. The robot pose should match reality and recover after the tags go out of view.
2. **Hold A** from several spots, including spots where the camera can't see the tags. It should
   still turn to face the hub, which is 24 in behind the wall, directly behind tag 10.
3. Strafe left and right while holding A. It should stay on target. If it lags, check the
   feedforward is working (`Aim/FeedforwardRadPerSec` should be nonzero while strafing).
4. Bring the logs home and look at `RobotPosesAccepted` / `Rejected`.

**Done when:** A aims to within ~2° from anywhere within ~12 ft, including briefly with no tag in
view.

## Session 4: shooter velocity control and distance table

1. **Flywheel alone (D-pad up),** no balls. Set `/Tuning/Shooter/ManualRPM` to 2000, 3000, 4000.
   - Spinning the wrong way? Flip `LAUNCH_INVERTED`.
   - Tune: set `kP = 0`, then adjust `kV` until the measured RPM settles near the target at each
     speed. Add a small `kS` if it falls short at low speed. Then raise `kP` until speed recovers
     quickly after a ball.
2. **Build the table:** for 4–6 distances (watch `Field/DistanceToHubMeters`), hold D-pad up,
   adjust ManualRPM until shots score, and write down distance → RPM.
3. Put the table into `ShooterConstants.DISTANCE_TO_RPM`. Copy the tuned gains into the defaults.

**Done when:** holding **A** and pulling RT scores from each of the table's distances.

## Session 5: PathPlanner autos

1. Install the **PathPlanner** GUI (Microsoft Store or GitHub releases) and open this project
   folder. Look at `Example - Back Up To Shot`.
2. **Run the example auto in the classroom** (Driver Station on Red, tags 10/9 up). The red
   version starts 0.62 m (24.5 in) from the wall to the robot's **center**, centered on tag 10, with
   the robot's **front** facing the wall. Place the robot there, select **Example - Back Up And
   Shoot**, and run Autonomous. It backs up about 1.3 m, turns the shooter to the hub, and shoots.
   (If your shooter is on the back, change the path's start/end rotation to 180° in the GUI so it
   doesn't have to turn around.)
3. If it doesn't stop where expected, check (in order): pose correct before starting? Robot config
   (mass, motor) right? Then the path-following PID in `Drive` (`PPHolonomicDriveController`).
4. Draw your own path in the GUI and add it to a new auto.

**Done when:** the example auto ends in the same spot (within ~10 cm) three times in a row.

## Session 6: drive to a spot (pathfinding) and putting it together

1. **Hold D-pad down:** the robot drives itself to the shooting spot, then aims. Let go to stop.
   Start slow (the pathfinding speed limit is in `RobotContainer.pathfindingConstraints`).
2. Move the shooting spot (`FieldConstants.blueShootingPosition`) to where your shooter is
   best.
3. Full sequence: D-pad down → hold A → pull RT.

**Done when:** you can go from anywhere in front of the hub to a scored ball without touching the
sticks.

---

## After that

- Paint the bumps, trenches and towers into the navgrid in the PathPlanner GUI.
- Shoot on the move (doc 02, "Where to go next").
- Practice on a real field at an offseason event, with all the tags in their real positions.
