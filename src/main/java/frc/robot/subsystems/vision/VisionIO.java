// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    /** The most recent camera frame (kept until a newer frame arrives). */
    public TargetObservation latestTargetObservation = TargetObservation.NONE;

    /**
     * Every camera frame received since the last robot loop, including frames where NO tag was
     * seen. Used to measure the detection rate (what % of frames actually saw a tag).
     */
    public TargetObservation[] targetObservations = new TargetObservation[0];

    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];

    /** Limelight hardware metrics: [cpu temp (C), cpu usage (%), ram usage (%), fps]. */
    public double[] hardwareMetrics = new double[0];
  }

  /**
   * One camera frame of simple "where is the tag in the image" data. Not used for pose estimation;
   * this is what the simple tx-based aiming uses.
   *
   * @param hasTarget Whether any tag was seen in this frame
   * @param tx Horizontal angle from the crosshair to the tag (positive = tag is to the RIGHT)
   * @param ty Vertical angle from the crosshair to the tag (positive = tag is UP)
   * @param timestamp When the image was captured (FPGA seconds, already latency-compensated)
   * @param tagId ID of the primary tag, or -1 if none
   * @param area Tag size as a percent of the image (0-100). Tiny values mean far away / few pixels.
   */
  public static record TargetObservation(
      boolean hasTarget, Rotation2d tx, Rotation2d ty, double timestamp, int tagId, double area) {
    public static final TargetObservation NONE =
        new TargetObservation(false, Rotation2d.kZero, Rotation2d.kZero, 0.0, -1, 0.0);
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /** Saves a still image on the camera for later review (Limelight snapshot). */
  public default void takeSnapshot() {}

  /**
   * Saves the last few seconds of video on the camera (Limelight 4 "Rewind" feature). Press this
   * right after the tag drops out and you get a video of exactly what the camera saw.
   */
  public default void captureRewind(double seconds) {}
}
