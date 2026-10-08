// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.aprilTagLayout;

  // Camera names, must match names configured on coprocessor.
  // "limelight" is the Limelight's default hostname/name.
  public static String camera0Name = "limelight";

  // Robot to camera transform: where the camera is on the robot, measured from the CENTER of the
  // robot at FLOOR level. +X = forward, +Y = left, +Z = up.
  //
  // WPILib angle convention: a camera tilted UP has a NEGATIVE pitch.
  //
  // The real Limelight does NOT read this. Enter the same numbers in the Limelight web UI
  // (Settings / "Robot-space camera pose"); this copy is used by the simulator.
  //
  // Limelight 3, mounted at the BACK-MIDDLE of the robot, facing FORWARD (so the robot's body is
  // in front of it), about 1.5 ft (18") up, tilted up about 15 degrees.
  //
  // TODO: X is a GUESS. Measure from the robot's center (middle of the four wheels) back to the
  // camera lens and put it here as a NEGATIVE number (behind center). Check the height and tilt
  // too (a phone level app on the camera works). Then type the SAME numbers into the Limelight
  // web UI, or the real robot and the simulator will disagree.
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(-10.0),
          0.0,
          Units.inchesToMeters(18.0),
          new Rotation3d(0.0, Units.degreesToRadians(-15.0), 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 // Camera 0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // How many seconds of frames to use when calculating the detection rate shown on the dashboard
  public static double detectionRateWindowSecs = 2.0;
}
