package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for the shooter. The Shooter subsystem only talks to this interface, so the
 * same subsystem code runs on the real robot (ShooterIOTalonFX), in simulation (ShooterIOSim), and
 * in log replay (an empty ShooterIO).
 */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean launchConnected = false;
    public double launchVelocityRpm = 0.0;
    public double launchAppliedVolts = 0.0;
    public double launchCurrentAmps = 0.0;

    public boolean transferConnected = false;
    public double transferAppliedVolts = 0.0;
    public double transferCurrentAmps = 0.0;
  }

  /** Reads sensors into the inputs object. Called once per loop. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Closed-loop flywheel velocity (the motor controller holds this speed on its own). */
  public default void setLaunchVelocity(double rpm) {}

  /** Open-loop flywheel output, -1 to 1 (like the old code's launchMotor.set()). */
  public default void setLaunchPercent(double percent) {}

  /** Open-loop transfer output, -1 to 1. */
  public default void setTransferPercent(double percent) {}

  /** Updates the flywheel velocity gains (kS/kV in volts per RPS, kP in volts per RPS of error). */
  public default void setLaunchGains(double kS, double kV, double kP) {}

  public default void stop() {}
}
