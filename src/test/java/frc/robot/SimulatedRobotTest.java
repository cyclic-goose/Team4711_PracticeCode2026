package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AimController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.AimingMath;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs the real robot code against the physics simulation, with simulated time, and checks that it
 * actually does what it should. Runs on any laptop with ./gradlew test (no robot needed).
 *
 * <p>If one of these fails after you change something, the change broke behavior, not just syntax.
 */
class SimulatedRobotTest {
  private static RobotContainer robot;

  @BeforeAll
  static void setUp() {
    robot = SimTestHelper.robot();
  }

  @AfterEach
  void cancelCommands() {
    SimTestHelper.reset(); // Let the robot settle between tests
  }

  private static void runFor(double seconds) {
    SimTestHelper.runFor(seconds);
  }

  private static void teleport(Pose2d pose) {
    SimTestHelper.teleport(pose);
  }

  private static double headingErrorToHubDegrees(Drive drive) {
    Rotation2d wanted =
        AimingMath.headingToFaceTarget(
            drive.getPose().getTranslation(),
            FieldConstants.getOurHubCenter(),
            AimController.AIM_SIDE);
    return wanted.minus(drive.getRotation()).getDegrees();
  }

  @Test
  void level2AimTurnsToFaceTheHub() {
    Drive drive = robot.getDrive();
    teleport(new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(150.0))); // Facing way off

    CommandScheduler.getInstance().schedule(robot.getAim().aimAtHub(() -> 0.0, () -> 0.0));
    runFor(2.0);

    assertEquals(0.0, headingErrorToHubDegrees(drive), 2.0);
    assertTrue(robot.getAim().isAimed());
  }

  @Test
  void level2AimStaysOnTargetWhileStrafing() {
    Drive drive = robot.getDrive();
    teleport(new Pose2d(2.0, 1.5, Rotation2d.kZero));

    // Half-speed strafe to the left (+Y) the whole time, which sweeps the hub's direction around
    CommandScheduler.getInstance().schedule(robot.getAim().aimAtHub(() -> 0.0, () -> 0.7));
    runFor(1.0); // Settle

    double worstErrorDegrees = 0.0;
    for (int i = 0; i < 40; i++) {
      runFor(0.02);
      worstErrorDegrees = Math.max(worstErrorDegrees, Math.abs(headingErrorToHubDegrees(drive)));
    }
    assertTrue(
        worstErrorDegrees < 4.0,
        "Aim fell behind while strafing; worst error " + worstErrorDegrees + " deg");
  }

  @Test
  void level1TxAimWorksDespiteLatencyAndDroppedFrames() {
    Drive drive = robot.getDrive();
    teleport(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(10.0)));
    Translation2d tagPosition = new Translation2d(4.0, 3.0);

    // A fake camera that behaves like a struggling Limelight: 25 fps, 60 ms latency, and it
    // DROPS 50% of frames (roughly what was seen on the real robot).
    var fakeCamera = new FakeTxCamera(drive, tagPosition, 0.06, 0.5);
    var fakeVision = new Vision((pose, timestamp, stdDevs) -> {}, fakeCamera);
    var txAim = new AimController(drive, fakeVision);

    CommandScheduler.getInstance().schedule(txAim.aimWithLimelightTx(() -> 0.0, () -> 0.0));
    runFor(2.5);

    Rotation2d wanted = AimingMath.bearing(drive.getPose().getTranslation(), tagPosition);
    assertEquals(0.0, wanted.minus(drive.getRotation()).getDegrees(), 1.5);
    CommandScheduler.getInstance().unregisterSubsystem(fakeVision);
  }

  @Test
  void pathfindingDrivesAroundTheHubToTheAimingSpot() throws Exception {
    Drive drive = robot.getDrive();
    // Start on the far side of the blue hub, so the path has to go around it
    teleport(new Pose2d(6.5, 4.0, Rotation2d.k180deg));
    var target = new Pose2d(FieldConstants.blueAimingSpot, Rotation2d.kZero);

    CommandScheduler.getInstance()
        .schedule(
            AutoBuilder.pathfindToPose(
                target,
                new com.pathplanner.lib.path.PathConstraints(
                    2.0, 2.0, Math.toRadians(360), Math.toRadians(540))));
    // The pathfinder plans on a background thread in REAL time, but this test runs simulated time
    // ~10x faster than real time, so give the planner a moment before the robot starts moving.
    Thread.sleep(500);
    double hubHalfWidth = 1.207 / 2.0;
    double robotHalfWidth = 0.4; // Bumpers are ~0.45; the navgrid adds extra margin beyond that
    for (int i = 0; i < 500; i++) {
      runFor(0.02);
      Thread.sleep(2);
      var position = drive.getPose().getTranslation();
      boolean insideHub =
          Math.abs(position.getX() - FieldConstants.blueHubCenter.getX())
                  < hubHalfWidth + robotHalfWidth
              && Math.abs(position.getY() - FieldConstants.blueHubCenter.getY())
                  < hubHalfWidth + robotHalfWidth;
      assertTrue(!insideHub, "Drove through the hub at " + position);
    }

    assertEquals(
        0.0,
        drive.getPose().getTranslation().getDistance(FieldConstants.blueAimingSpot),
        0.15,
        "Ended at " + drive.getPose());
  }

  /** Fake Limelight: reports tx toward a tag, with latency and random dropped frames. */
  private static class FakeTxCamera implements VisionIO {
    private final Drive drive;
    private final Translation2d tag;
    private final double latencySecs;
    private final double dropChance;
    private final Random random = new Random(4711); // Same "random" drops every run
    private final Deque<double[]> headingHistory = new ArrayDeque<>(); // [time, heading rad]
    private TargetObservation latest = TargetObservation.NONE;
    private int loopCount = 0;

    FakeTxCamera(Drive drive, Translation2d tag, double latencySecs, double dropChance) {
      this.drive = drive;
      this.tag = tag;
      this.latencySecs = latencySecs;
      this.dropChance = dropChance;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      double now = Timer.getTimestamp();
      headingHistory.addLast(new double[] {now, drive.getRotation().getRadians()});
      while (headingHistory.size() > 50) {
        headingHistory.removeFirst();
      }

      inputs.connected = true;
      inputs.targetObservations = new TargetObservation[0];
      loopCount++;
      if (loopCount % 2 == 0) { // 25 fps
        double captureTime = now - latencySecs;
        double headingThen = headingAt(captureTime);
        boolean dropped = random.nextDouble() < dropChance;
        // tx is positive when the tag is to the RIGHT, i.e. opposite sign of the angle error
        double bearing = AimingMath.bearing(drive.getPose().getTranslation(), tag).getRadians();
        double tx = -new Rotation2d(bearing).minus(new Rotation2d(headingThen)).getRadians();
        latest =
            new TargetObservation(
                !dropped,
                dropped ? Rotation2d.kZero : new Rotation2d(tx),
                Rotation2d.kZero,
                captureTime,
                dropped ? -1 : 1,
                dropped ? 0.0 : 0.5);
        inputs.targetObservations = new TargetObservation[] {latest};
      }
      inputs.latestTargetObservation = latest;
    }

    private double headingAt(double time) {
      double best = drive.getRotation().getRadians();
      for (double[] sample : headingHistory) {
        if (sample[0] <= time) {
          best = sample[1];
        }
      }
      return best;
    }
  }
}
