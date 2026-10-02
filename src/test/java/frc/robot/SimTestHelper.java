package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Shared setup for the simulation tests: ONE simulated robot for all test classes (creating a
 * second would register a second drivetrain and camera), plus helpers to run the robot loop.
 */
final class SimTestHelper {
  private static RobotContainer robot;

  private SimTestHelper() {}

  /** The simulated robot, created on first use. Driver Station: enabled, teleop, Blue 1. */
  static synchronized RobotContainer robot() {
    if (robot == null) {
      assertTrue(HAL.initialize(500, 0));
      SimHooks.pauseTiming(); // Time only moves when we call stepTiming()
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      DriverStation.silenceJoystickConnectionWarning(true); // No controller in tests
      robot = new RobotContainer();
    }
    return robot;
  }

  /** Runs the robot loop (50 times per second of simulated time). */
  static void runFor(double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
  }

  /** Schedules a command and runs the robot until it finishes (or the time limit is hit). */
  static void runUntilFinished(Command command, double maxSeconds) {
    CommandScheduler.getInstance().schedule(command);
    for (int i = 0; i < Math.round(maxSeconds / 0.02) && command.isScheduled(); i++) {
      runFor(0.02);
    }
  }

  /**
   * Moves the simulated robot to a new pose. (Camera frames captured before the move are ignored by
   * Drive.addVisionMeasurement, so they can't drag the robot back.)
   */
  static void teleport(Pose2d pose) {
    robot().getDrive().setPose(pose);
  }

  /** Changes the Driver Station alliance (what a real DS / the field would tell the robot). */
  static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  /** Cancels everything and lets the robot come to a stop. Call between tests. */
  static void reset() {
    CommandScheduler.getInstance().cancelAll();
    runFor(0.5);
  }
}
