package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public final class ShooterConstants {
  private ShooterConstants() {}

  /**
   * Distance (meters, robot center to hub center) -> flywheel speed (RPM).
   *
   * <p>The robot interpolates between entries, e.g. halfway between 2.0 m and 3.0 m it uses the RPM
   * halfway between those two rows. Past the ends it uses the closest end value.
   *
   * <p>TODO: these are placeholders. Fill them in on the robot (see docs/04-robot-session-plans.md,
   * "Build the shooter table"): park at a distance, adjust /Tuning/Shooter/ManualRPM until shots go
   * in, write down (distance, RPM), repeat at 4-6 distances.
   */
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.5, 2500.0),
          Map.entry(2.5, 2900.0),
          Map.entry(3.5, 3300.0),
          Map.entry(4.5, 3700.0),
          Map.entry(5.5, 4100.0));

  /** How close (RPM) the flywheel has to be to its target to count as "ready to shoot". */
  public static final double RPM_TOLERANCE = 75.0;

  /** Flywheel speed while just waiting, so it doesn't have to spin up from zero. 0 = off. */
  public static final double IDLE_RPM = 0.0;

  /**
   * Motor rotations per flywheel rotation. 1.0 = direct drive. TODO: set to the real gear ratio
   * (e.g. 18T motor pulley driving a 36T flywheel pulley = 2.0).
   */
  public static final double LAUNCH_GEAR_RATIO = 1.0;

  /** Set true if positive output spins the flywheel the wrong way. */
  public static final boolean LAUNCH_INVERTED = false;

  public static final boolean TRANSFER_INVERTED = false;

  // Starting gains for Phoenix 6 VelocityVoltage (units: volts per rotation-per-second).
  // kV ~= 12 V / free speed in RPS. A Kraken X60 is ~100 RPS free, so ~0.12.
  public static final double DEFAULT_KS = 0.15;
  public static final double DEFAULT_KV = 0.12;
  public static final double DEFAULT_KP = 0.15;

  // Simulation only: how heavy the flywheel is to spin up. A guess is fine.
  public static final double SIM_MOI_KG_M2 = 0.004;
}
