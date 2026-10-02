// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * IO implementation for real Limelight hardware.
 *
 * <p>The Limelight publishes its results to NetworkTables under the table named after the camera
 * (default "limelight"). Key reference: https://docs.limelightvision.io (Complete NetworkTables
 * API).
 *
 * <p>The camera's position on the robot ("robot-space camera pose") is configured in the Limelight
 * web UI, not here. Keep it in sync with {@link VisionConstants#robotToCamera0}, which the
 * simulator uses.
 */
public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleArraySubscriber t2dSubscriber;
  private final DoubleArraySubscriber hardwareSubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private TargetObservation latestTargetObservation = TargetObservation.NONE;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    t2dSubscriber = table.getDoubleArrayTopic("t2d").subscribe(new double[] {});
    hardwareSubscriber = table.getDoubleArrayTopic("hw").subscribe(new double[] {});
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Read every frame of simple targeting data since the last loop. "t2d" is published once per
    // camera frame and holds matched values, so tx and the tag ID always come from the same image:
    // [valid, count, tl (ms), cl (ms), tx, ty, txnc, tync, ta, tid, ...]
    List<TargetObservation> targetObservations = new LinkedList<>();
    for (var rawSample : t2dSubscriber.readQueue()) {
      double[] t2d = rawSample.value;
      if (t2d.length < 10) continue;
      double totalLatencySeconds = (t2d[2] + t2d[3]) * 1.0e-3;
      targetObservations.add(
          new TargetObservation(
              t2d[0] == 1.0,
              Rotation2d.fromDegrees(t2d[4]),
              Rotation2d.fromDegrees(t2d[5]),
              rawSample.timestamp * 1.0e-6 - totalLatencySeconds,
              t2d[0] == 1.0 ? (int) t2d[9] : -1,
              t2d[8]));
    }
    inputs.targetObservations = targetObservations.toArray(new TargetObservation[0]);
    if (!targetObservations.isEmpty()) {
      latestTargetObservation = targetObservations.get(targetObservations.size() - 1);
    }
    inputs.latestTargetObservation = latestTargetObservation;

    inputs.hardwareMetrics = hardwareSubscriber.get();

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  @Override
  public void takeSnapshot() {
    // Incrementing the "snapshot" value tells the Limelight to save an image
    var entry = table.getEntry("snapshot");
    entry.setDouble(entry.getDouble(0.0) + 1.0);
  }

  @Override
  public void captureRewind(double seconds) {
    // [counter, duration]: incrementing the counter saves the last "duration" seconds of video
    var entry = table.getEntry("capture_rewind");
    double[] current = entry.getDoubleArray(new double[] {});
    double counter = current.length > 0 ? current[0] : 0.0;
    entry.setDoubleArray(new double[] {counter + 1.0, Math.min(seconds, 165.0)});
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
