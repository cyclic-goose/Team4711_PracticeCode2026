// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonSRX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *
 * <p>Controller map (Xbox controller on port 0) is listed in README.md.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Intake intake;
  private final AimController aim;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Speed limits for on-the-fly pathfinding (m/s, m/s², rad/s, rad/s²). Start slow!
  private final PathConstraints pathfindingConstraints =
      new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        shooter = new Shooter(new ShooterIOTalonFX());
        intake = new Intake(new IntakeIOTalonSRX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    aim = new AimController(drive, vision);

    // Named commands MUST be registered before the auto chooser is built, because building it
    // loads the .auto files, and those refer to commands by name.
    registerNamedCommands();

    // Set up auto routines (every .auto file in src/main/deploy/pathplanner/autos shows up here)
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Commands that PathPlanner autos can call by name (in the GUI: "Named Command"). */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", shootWhenReady().withTimeout(5.0));
    NamedCommands.registerCommand("SpinUp", shooter.spinUpForDistance(this::getDistanceToHub));
    NamedCommands.registerCommand(
        "AimAtHub", aim.aimAtHub(() -> 0.0, () -> 0.0).until(aim::isAimed).withTimeout(2.0));
    NamedCommands.registerCommand(
        "RunIntake", Commands.startEnd(() -> intake.setFeedPercent(0.65), intake::stop, intake));
  }

  /** Use this method to define your button->command mappings. Controller map is in README.md. */
  private void configureButtonBindings() {
    // ---------------- Driving ----------------
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Start: "the robot is facing away from me right now". Sets the heading to match the
    // alliance (0° on blue, 180° on red). Field-relative driving AND MegaTag2 both need this
    // to be right, so press it whenever you place the robot.
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                FieldConstants.isRedAlliance()
                                    ? Rotation2d.k180deg
                                    : Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // X: lock wheels in an X so the robot is hard to push
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // ---------------- Aiming ----------------
    // Y (hold): LEVEL 1 aim. Turns to center whatever tag the Limelight sees (the fixed
    // version of the old "center on tag").
    controller
        .y()
        .whileTrue(
            aim.aimWithLimelightTx(() -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // A (hold): LEVEL 2 "shoot mode". Aims at our hub using the field pose AND spins the
    // flywheel to the right speed for the distance. Pull the right trigger to feed.
    controller
        .a()
        .whileTrue(
            Commands.parallel(
                aim.aimAtHub(() -> -controller.getLeftY(), () -> -controller.getLeftX()),
                shooter.spinUpForDistance(this::getDistanceToHub)));

    // D-pad up (hold): shooter TUNING mode. Same as A, but the flywheel runs at
    // /Tuning/Shooter/ManualRPM so you can find the right speed for each distance.
    controller
        .povUp()
        .whileTrue(
            Commands.parallel(
                aim.aimAtHub(() -> -controller.getLeftY(), () -> -controller.getLeftX()),
                shooter.spinUpManual()));

    // D-pad down (hold): drive itself to the shooting spot (PathPlanner pathfinding), then aim.
    // Let go to cancel instantly. pathfindToPoseFlipped mirrors the blue spot for red.
    controller
        .povDown()
        .whileTrue(
            AutoBuilder.pathfindToPoseFlipped(
                    new Pose2d(
                        FieldConstants.blueShootingPosition,
                        Rotation2d.kZero.minus(AimController.SHOOTER_FACING)),
                    pathfindingConstraints)
                .andThen(aim.aimAtHub(() -> 0.0, () -> 0.0)));

    // D-pad right (hold): run the shooter transfer motor. Direction copied from the old
    // commented-out code (-0.55). TODO: verify direction/whether your shooter needs it.
    controller.povRight().whileTrue(shooter.runTransfer(-0.55));

    // Rumble the controller when aimed AND the flywheel is at speed: safe to shoot.
    new Trigger(() -> aim.isAimed() && shooter.atSpeed())
        .whileTrue(
            Commands.startEnd(
                    () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.4),
                    () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))
                .ignoringDisable(true));

    // ---------------- Intake (same as the old code) ----------------
    // Left bumper = deploy, right bumper = retract, right trigger = rollers in,
    // left trigger = rollers out. B toggles "locked" (retract and ignore controls).
    intake.setDefaultCommand(
        intake.manualControl(
            () -> controller.leftBumper().getAsBoolean(),
            () -> controller.rightBumper().getAsBoolean(),
            controller::getRightTriggerAxis,
            controller::getLeftTriggerAxis));
    controller.b().onTrue(Commands.runOnce(intake::toggleLocked));

    // ---------------- Vision debugging ----------------
    // Back: save a Limelight snapshot AND the last 20 seconds of video (Limelight 4 Rewind).
    // Press it right after the tag drops out, then look at the files in the Limelight web UI.
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      vision.takeSnapshot();
                      vision.captureRewind(20.0);
                    })
                .ignoringDisable(true));
  }

  /**
   * Aim at the hub, spin up for the distance, and feed once BOTH are ready (or after 2 s). Used by
   * autos via the "Shoot" named command.
   */
  private Command shootWhenReady() {
    Command feed =
        Commands.startEnd(() -> intake.setFeedPercent(0.75), intake::stop, intake).withTimeout(1.5);
    return Commands.deadline(
        Commands.sequence(
            Commands.waitUntil(() -> aim.isAimed() && shooter.atSpeed()).withTimeout(2.0), feed),
        aim.aimAtHub(() -> 0.0, () -> 0.0),
        shooter.spinUpForDistance(this::getDistanceToHub));
  }

  /** Distance from the robot's center to the center of our hub, in meters. */
  public double getDistanceToHub() {
    return drive.getPose().getTranslation().getDistance(FieldConstants.getOurHubCenter());
  }

  /** Called every loop from Robot.robotPeriodic(), for values that are handy on a dashboard. */
  public void updateDashboard() {
    Logger.recordOutput("Field/OurHubCenter", FieldConstants.getOurHubCenter());
    Logger.recordOutput("Field/DistanceToHubMeters", getDistanceToHub());
    Logger.recordOutput("Field/TableRPMForDistance", Shooter.rpmForDistance(getDistanceToHub()));
    Logger.recordOutput("Aim/ReadyToShoot", aim.isAimed() && shooter.atSpeed());
  }

  // Accessors for the simulation tests in src/test (package-private on purpose)
  Drive getDrive() {
    return drive;
  }

  AimController getAim() {
    return aim;
  }

  Shooter getShooter() {
    return shooter;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
