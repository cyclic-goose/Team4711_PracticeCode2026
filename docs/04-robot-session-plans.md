# 04: Robot session plans (Tue/Thu, 2 hours)

The focus is **Limelight odometry** and **path planning**. Each session has **one main goal**, a
short checklist, and a "done when". If a session finishes early, start the next one. If it
doesn't, repeat it. Between sessions: put the measurements and tuned values into the code, run
`./gradlew test`, and try changes in sim.

## Every session

- [ ] **Before leaving home:** `./gradlew build` passes; commit and push so you can roll back.
- [ ] **USB stick** in the roboRIO (that's where the logs go).
- [ ] AdvantageScope layout ready; laptop charged.
- [ ] **Safety:** teleop starts at **50% speed** (`/Tuning/Drive/TeleopSpeedScale`). On the
      Driver Station, **Enter = disable** and **Space = emergency stop**. Keep a hand near them.
      Clear the floor before autos or pathfinding.
- [ ] **After:** download the log (AdvantageScope → File → Download Logs) and write a few notes on
      what changed and what you saw.

The old code is still on the `PostSeasonCelebrationSpeed` branch of the old repo. If something
goes badly wrong, you can redeploy that.

---

## Before session 1 (at home)

- [ ] **Bench test the Limelight** ([01](01-limelight-bench-test.md)). Bring it home next class.
      Print proper tags (10, 9, 26, 25).
- [ ] Run the simulation and drive around in it ([03](03-simulation-and-logs.md)). Run the
      `Classroom - Tour` auto in sim and watch it in AdvantageScope.
- [ ] Install the PathPlanner GUI and open the project ([05](05-pathplanner-guide.md), Part B).

## Session 1: new code running, take measurements

1. **Robot on blocks.** Deploy. The drivetrain uses your existing `TunerConstants`, so the
   modules should behave exactly as before. Check each wheel drives and steers.
2. **Measure** (write everything down):
   - Camera: forward / sideways / up from the **robot center at floor level**, and its tilt.
   - Robot weight with bumpers and battery; bumper outer length and width.
3. **Floor, 50% speed:** point the robot away from you, press **Start**, and drive. Forward on the
   stick should move away from you whatever way the robot is turned.
4. **Intake:** bumpers, triggers and B lock should work exactly like before.
5. **Wheel radius:** pick **Drive Wheel Radius Characterization** in the auto chooser, enable
   Autonomous, and let the robot spin slowly in place for 2–3 turns, then disable. The result
   prints in the Driver Station console. Put it in `TunerConstants.kWheelRadius`. Path following
   and odometry both depend on this number.
6. **Limelight:** apply your bench-test settings. Set IP `10.47.11.11`, name `limelight`, and the
   2026 field map. Enter the camera pose. If you can, move power off the VRM 500 mA output (see
   doc 01).
7. **Tape up tags 10 and 9** on a wall (doc 01, "Testing the field-position features in a
   classroom"). Leave them there if you're allowed to.

**Done when:** the robot drives field-relative, the intake works, and you have the measurements.

**At home:** fill in the `TODO`s (`robotToCamera0`, mass in `Drive` **and** `settings.json`,
bumper size) and run the tests.

## Session 2: vision and Level 1 aim (Y)

1. With the Driver Station on **Red**, park the robot facing the tags at 3, 6 and 10 ft. Check
   `Vision/Camera0/DetectionRatePercent`. It should be near 100%.
2. **Level 1 aim (hold Y):**
   - Robot still, pointed 30° away from the tags → it should snap to them.
   - Hold Y while strafing slowly → it should keep facing the tag.
   - Tune `/Tuning/Aim/kP` and `kD` (doc 02, "The heading controller"). Watch
     `Aim/ErrorDegrees`.
3. **Any time the tag drops, press Back** (snapshot + 20 s video).

**Done when:** detection holds to about 12 ft and Y snaps to the tag in under ~0.5 s without
wobbling.

## Session 3: Limelight odometry quality

The goal: `Odometry/Robot` matches reality everywhere you'll drive. Everything in sessions 4–5
depends on this.

1. **Accuracy check.** Mark 3–4 spots on the floor with tape and measure each spot's field
   position. Tag 10 is at (12.52, 4.03), facing +X, so 2 m straight out from tag 10 is
   (14.52, 4.03); 2 m out and 1 m to the left as you face the wall is (14.52, 3.03). Park on each
   spot and compare with `Odometry/Robot`. A few cm off is good. A consistent offset in one
   direction usually means the **camera pose** in the Limelight is wrong.
2. **Heading check.** Is the heading in `Odometry/Robot` right? It should be 180° when the robot
   faces the wall on Red. If not, press **Start** facing away from the wall and recheck.
3. **Drive around** for a minute with the tags in and out of view and watch the 3D field. The pose
   should stay smooth and snap back if it drifted when the tags come back. Note any jumps.
4. **Hold A (Level 2 aim)** from a few spots, including spots where the camera can't see the tags.
   It should face the hub center (24 in behind tag 10). If it points somewhere else, the pose is
   wrong.
5. **At home:** look at `Vision/Camera0/RobotPosesAccepted` / `Rejected` in the log. Are good poses
   being rejected, or bad ones accepted? `VisionConstants` (ambiguity limit, std devs) is where to
   adjust.

**Done when:** all marked spots read within ~5 cm, and A aims correctly even with no tag in view.

## Session 4: first PathPlanner autos

1. **Place the robot** at the classroom start: centered on tag 10, **56 in** from the wall to the
   robot's **center**, front facing the wall. Driver Station on **Red**.
2. Run **`Classroom - Loop`** (Autonomous). Watch `Odometry/TrajectorySetpoint` (ghost) and
   `PathPlanner/TranslationErrorMeters`.
3. **Hold D-pad up** to drive back to the start. Run it again. Each run should end in the same
   place.
4. If it lags, wiggles or misses: follow doc 05, B8 (pose first, then robot config, then path
   PID).
5. Run **`Classroom - Tour`** (loop, orbit facing the hub, return, aim).

**Done when:** `Classroom - Loop` ends within ~10 cm of the start three runs in a row, and the
Tour runs cleanly.

## Session 5: your own paths, and pathfinding

1. At home, draw **your own path and auto** in the PathPlanner GUI (doc 05, Part B). Run
   `./gradlew test`, which checks it automatically, and try it in sim.
2. On the robot: deploy, then run it.
3. **Hold D-pad down:** the robot drives itself to the aiming spot in front of the hub, then aims.
   Let go to stop. Start slow (`RobotContainer.pathfindingConstraints`).
4. Try `pathfindThenFollowPath` for a precise final approach (doc 05, A6).

**Done when:** you can draw a new path at home, check it in sim, and have it run correctly on the
robot the same day.

---

## After that

- Paint the bumps, trenches and towers into the navgrid in the PathPlanner GUI.
- Event markers that run real mechanisms (intake) during a path.
- Practice on a real field at an offseason event, with all the tags in their real positions.
