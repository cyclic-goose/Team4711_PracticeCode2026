// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /**
   * When true, every {@link frc.robot.util.TunableNumber} shows up under "/Tuning" in NetworkTables
   * so gains can be changed live from AdvantageScope or Elastic without redeploying. Leave this on
   * while learning/tuning, turn it off for competitions.
   */
  public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** CAN IDs for the non-drivetrain mechanisms (drivetrain IDs live in TunerConstants). */
  public static final class CanIds {
    public static final int FEED_MOTOR_1 = 15; // Intake roller (TalonSRX)
    public static final int FEED_MOTOR_2 = 16; // Intake roller (TalonSRX)
    public static final int FEED_MOVE_MOTOR = 10; // Intake deploy/retract (TalonSRX)
    public static final int TRANSFER_MOTOR = 17; // Shooter transfer (TalonFX), shooter not in use
    public static final int LAUNCH_MOTOR = 18; // Shooter flywheel (TalonFX), shooter not in use
  }

  /** roboRIO DIO ports. */
  public static final class DioPorts {
    public static final int INTAKE_LIMIT_FRONT = 0;
    public static final int INTAKE_LIMIT_BACK = 1;
  }
}
