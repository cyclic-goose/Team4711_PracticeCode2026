// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * IO implementation for physics sim using PhotonVision simulator.
 *
 * <p>Even though the real robot uses a Limelight, PhotonVision's simulator is a great stand-in: it
 * knows where every tag on the field is, figures out which ones the camera could see from the
 * simulated robot pose, and produces realistic (noisy, delayed) detections. That lets you test
 * vision fusion and aiming on your laptop.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera, roughly matching a Limelight 4 (1280x800, ~82 x 56 degree field of view)
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(91.0));
    cameraProperties.setCalibError(0.35, 0.10);
    cameraProperties.setFPS(30.0);
    cameraProperties.setAvgLatencyMs(25.0);
    cameraProperties.setLatencyStdDevMs(5.0);
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);

    // The simulated video streams are nice to look at (http://localhost:1182) but cost CPU.
    // Set these to true if you want to see what the simulated camera sees.
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
