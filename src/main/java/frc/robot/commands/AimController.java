package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AimingMath;
import frc.robot.util.TunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Commands that let the driver keep translating with the left stick while the code controls
 * rotation to aim.
 *
 * <p>There are two ways to aim, and it's worth understanding both:
 *
 * <h2>Level 1: aim with Limelight tx ({@link #aimWithLimelightTx})</h2>
 *
 * Only needs the camera to see a tag; no field map, no camera position, no pose estimation. Each
 * time a NEW camera frame arrives, tx is turned into a target heading ("the tag is 12° right of
 * where I was pointing when this picture was taken"). Then the GYRO, which updates every loop with
 * almost no delay, is used to turn to that heading. This is far more stable than feeding tx
 * straight into a PID controller (what the old code did), because:
 *
 * <ul>
 *   <li>The camera is slow and delayed; the gyro is fast and instant. Control with the fast sensor.
 *   <li>A dropped frame doesn't matter: the target heading is simply held until the next frame.
 * </ul>
 *
 * <h2>Level 2: aim at a field position ({@link #aimAtHub})</h2>
 *
 * Uses the robot's estimated field position (wheel odometry + gyro, corrected by vision). The
 * target is a fixed point on the field (the hub center), so the robot can aim even when NO tag is
 * visible, and it knows the distance for setting shooter speed. This is what most good teams do. It
 * requires the camera position to be set correctly in the Limelight and a correct starting heading.
 *
 * <p>Both share the same heading controller, tuned live from the dashboard under /Tuning/Aim/.
 */
public class AimController {
  /**
   * Which way the shooter points relative to the robot's front. Rotation2d.kZero means the shooter
   * shoots out the front of the robot, Rotation2d.k180deg means out the back.
   *
   * <p>TODO: set this to match your robot.
   */
  public static final Rotation2d SHOOTER_FACING = Rotation2d.kZero;

  // Heading controller gains. Units: kP is (rad/s of turning) per (rad of error).
  // Start with kD = 0. Raise kP until it snaps to the target quickly; if it overshoots/wobbles,
  // add a little kD or lower kP.
  private final TunableNumber kP = new TunableNumber("Aim/kP", 5.0);
  private final TunableNumber kD = new TunableNumber("Aim/kD", 0.2);
  private final TunableNumber maxOmega = new TunableNumber("Aim/MaxOmegaRadPerSec", 6.0);
  private final TunableNumber toleranceDeg = new TunableNumber("Aim/ToleranceDeg", 2.0);
  // Only trust a tx reading if the frame is newer than this (the pose history only goes back ~1.5s)
  private final TunableNumber maxFrameAgeSecs = new TunableNumber("Aim/MaxFrameAgeSecs", 0.5);

  private final Drive drive;
  private final Vision vision;
  private final PIDController controller;

  private double lastErrorRad = Double.POSITIVE_INFINITY;
  private double lastRunTimestamp = Double.NEGATIVE_INFINITY;

  public AimController(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    controller = new PIDController(kP.get(), 0.0, kD.get());
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * True if an aim command ran within the last 0.1 s and the heading error is inside the tolerance.
   * Use this to only feed a ball when the robot is actually pointed at the target.
   */
  public boolean isAimed() {
    return Timer.getTimestamp() - lastRunTimestamp < 0.1
        && Math.abs(lastErrorRad) < Math.toRadians(toleranceDeg.get());
  }

  /**
   * Level 1: turn to center the tag that the Limelight sees, while the driver controls translation.
   *
   * @param xSupplier Forward stick input (-1 to 1)
   * @param ySupplier Left stick input (-1 to 1)
   */
  public Command aimWithLimelightTx(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Command state lives in this small object so each run starts fresh.
    var state =
        new Object() {
          double lastFrameTimestamp = 0.0;
          Rotation2d targetHeading = null; // null = haven't seen a tag yet
        };

    return Commands.run(
            () -> {
              var frame = vision.getLatestTarget(0);
              boolean isNewFrame = frame.timestamp() > state.lastFrameTimestamp;
              boolean isRecent = Timer.getTimestamp() - frame.timestamp() < maxFrameAgeSecs.get();

              if (isNewFrame && isRecent && frame.hasTarget()) {
                Rotation2d headingAtCapture = drive.getRotationAt(frame.timestamp());
                state.targetHeading = AimingMath.headingFromTx(headingAtCapture, frame.tx());
              }
              if (isNewFrame) {
                state.lastFrameTimestamp = frame.timestamp();
              }

              ChassisSpeeds fieldSpeeds =
                  DriveCommands.joystickToFieldSpeeds(
                      drive, xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega = 0.0;
              if (state.targetHeading != null) {
                omega = calculateOmega(state.targetHeading, 0.0);
                Logger.recordOutput("Aim/TargetHeading", state.targetHeading);
              }
              runFieldRelative(fieldSpeeds, omega);

              Logger.recordOutput("Aim/Mode", "LimelightTx");
              Logger.recordOutput("Aim/HasTargetHeading", state.targetHeading != null);
              Logger.recordOutput("Aim/FrameAgeSecs", Timer.getTimestamp() - frame.timestamp());
            },
            drive)
        .beforeStarting(
            () -> {
              state.lastFrameTimestamp = 0.0;
              state.targetHeading = null;
              controller.reset();
            })
        .finallyDo(() -> Logger.recordOutput("Aim/Mode", "None"));
  }

  /**
   * Level 2: point the shooter at the center of our hub using the robot's field position, while the
   * driver controls translation.
   */
  public Command aimAtHub(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return aimAtPoint(xSupplier, ySupplier, FieldConstants::getOurHubCenter);
  }

  /** Level 2: point the shooter at any field position while the driver translates. */
  public Command aimAtPoint(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Translation2d> target) {
    return Commands.run(
            () -> {
              Translation2d robot = drive.getPose().getTranslation();
              Translation2d targetPosition = target.get();
              Rotation2d targetHeading =
                  AimingMath.headingToFaceTarget(robot, targetPosition, SHOOTER_FACING);

              ChassisSpeeds fieldSpeeds =
                  DriveCommands.joystickToFieldSpeeds(
                      drive, xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Feedforward: how fast the target's direction is sweeping because we're driving.
              // Uses the COMMANDED velocity (smooth, no delay) rather than the measured one.
              double feedforward =
                  AimingMath.bearingRateRadPerSec(
                      robot,
                      targetPosition,
                      fieldSpeeds.vxMetersPerSecond,
                      fieldSpeeds.vyMetersPerSecond);

              runFieldRelative(fieldSpeeds, calculateOmega(targetHeading, feedforward));

              Logger.recordOutput("Aim/Mode", "FieldPose");
              Logger.recordOutput("Aim/TargetHeading", targetHeading);
              Logger.recordOutput("Aim/TargetPosition", targetPosition);
              Logger.recordOutput("Aim/FeedforwardRadPerSec", feedforward);
              Logger.recordOutput("Aim/DistanceMeters", robot.getDistance(targetPosition));
            },
            drive)
        .beforeStarting(controller::reset)
        .finallyDo(() -> Logger.recordOutput("Aim/Mode", "None"));
  }

  /** Heading PID + feedforward, clamped to a safe turn rate. */
  private double calculateOmega(Rotation2d targetHeading, double feedforwardRadPerSec) {
    int id = hashCode();
    // Single "|" on purpose so both values get checked (and remembered) every time
    if (kP.hasChanged(id) | kD.hasChanged(id)) {
      controller.setPID(kP.get(), 0.0, kD.get());
    }

    Rotation2d currentHeading = drive.getRotation();
    lastErrorRad = targetHeading.minus(currentHeading).getRadians();
    lastRunTimestamp = Timer.getTimestamp();

    double omega =
        feedforwardRadPerSec
            + controller.calculate(currentHeading.getRadians(), targetHeading.getRadians());
    omega = MathUtil.clamp(omega, -maxOmega.get(), maxOmega.get());

    Logger.recordOutput("Aim/ErrorDegrees", Math.toDegrees(lastErrorRad));
    Logger.recordOutput("Aim/IsAimed", isAimed());
    Logger.recordOutput("Aim/OmegaRadPerSec", omega);
    return omega;
  }

  private void runFieldRelative(ChassisSpeeds fieldSpeeds, double omegaRadPerSec) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, omegaRadPerSec),
            drive.getRotation()));
  }
}
