// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Reads AprilTag results from the camera(s), throws out bad pose estimates, and sends the good ones
 * to the drivetrain's pose estimator (the "VisionConsumer").
 *
 * <p>Useful things this logs for debugging (open them in AdvantageScope):
 *
 * <ul>
 *   <li>Vision/Camera0/DetectionRatePercent - % of camera frames that saw a tag. If this is well
 *       under 100 while the tag is clearly in view, detection is the problem (tag print, lighting,
 *       exposure), not your code.
 *   <li>Vision/Camera0/MeasuredFPS - camera frames actually arriving per second
 *   <li>Vision/Camera0/RobotPosesAccepted / Rejected - 3D poses you can overlay on the field
 *   <li>Vision/SecondsSinceAcceptedPose - how stale the vision correction is
 * </ul>
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Deque<TargetObservation>[] frameHistory;
  private double lastAcceptedPoseTimestamp = Double.NEGATIVE_INFINITY;

  @SuppressWarnings("unchecked")
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize detection-rate history
    this.frameHistory = new Deque[io.length];
    for (int i = 0; i < io.length; i++) {
      frameHistory[i] = new ArrayDeque<>();
    }
  }

  /**
   * Returns the most recent simple targeting result (tx/ty/tag ID) from a camera. Check hasTarget()
   * before using tx, and use timestamp() to tell whether it's a NEW frame.
   */
  public TargetObservation getLatestTarget(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation;
  }

  /** Seconds since vision last corrected the robot's pose (infinite if never). */
  public double getSecondsSinceAcceptedPose() {
    return Timer.getTimestamp() - lastAcceptedPoseTimestamp;
  }

  /** Saves a snapshot on every camera. */
  public void takeSnapshot() {
    for (var cameraIO : io) {
      cameraIO.takeSnapshot();
    }
  }

  /** Saves the last few seconds of video on every camera (Limelight 4 Rewind). */
  public void captureRewind(double seconds) {
    for (var cameraIO : io) {
      cameraIO.captureRewind(seconds);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Update detection statistics
      logDetectionStats(cameraIndex);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations.
        // A standard deviation is "how much do I trust this measurement" in meters/radians:
        // SMALL = trust a lot, LARGE = trust a little. Trust drops quickly with distance (squared)
        // and rises with more tags in view.
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        lastAcceptedPoseTimestamp = Math.max(lastAcceptedPoseTimestamp, observation.timestamp());
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/SecondsSinceAcceptedPose", getSecondsSinceAcceptedPose());
  }

  /** Tracks what fraction of recent camera frames actually saw a tag. */
  private void logDetectionStats(int cameraIndex) {
    var history = frameHistory[cameraIndex];
    for (var frame : inputs[cameraIndex].targetObservations) {
      history.addLast(frame);
    }
    double cutoff = Timer.getTimestamp() - detectionRateWindowSecs;
    while (!history.isEmpty() && history.peekFirst().timestamp() < cutoff) {
      history.removeFirst();
    }

    int framesWithTarget = 0;
    for (var frame : history) {
      if (frame.hasTarget()) {
        framesWithTarget++;
      }
    }

    String prefix = "Vision/Camera" + Integer.toString(cameraIndex);
    Logger.recordOutput(
        prefix + "/DetectionRatePercent",
        history.isEmpty() ? 0.0 : 100.0 * framesWithTarget / history.size());
    Logger.recordOutput(prefix + "/MeasuredFPS", history.size() / detectionRateWindowSecs);
    Logger.recordOutput(
        prefix + "/HasTarget", inputs[cameraIndex].latestTargetObservation.hasTarget());
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
