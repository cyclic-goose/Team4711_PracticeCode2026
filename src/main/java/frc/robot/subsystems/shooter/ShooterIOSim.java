package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulated shooter: a physics model of a flywheel. It copies what the TalonFX's VelocityVoltage
 * does (kS + kV feedforward plus kP feedback) so gains that work here are a reasonable starting
 * point on the real robot.
 */
public class ShooterIOSim implements ShooterIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60(1);

  private final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              MOTOR, ShooterConstants.SIM_MOI_KG_M2, ShooterConstants.LAUNCH_GEAR_RATIO),
          MOTOR);

  private boolean closedLoop = false;
  private double targetRps = 0.0;
  private double openLoopVolts = 0.0;
  private double transferVolts = 0.0;
  private double kS = ShooterConstants.DEFAULT_KS;
  private double kV = ShooterConstants.DEFAULT_KV;
  private double kP = ShooterConstants.DEFAULT_KP;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double rps = flywheelSim.getAngularVelocityRPM() / 60.0;
    double volts = openLoopVolts;
    if (closedLoop) {
      volts =
          (targetRps == 0.0 ? 0.0 : kS * Math.signum(targetRps))
              + kV * targetRps
              + kP * (targetRps - rps);
    }
    volts = MathUtil.clamp(volts, -12.0, 12.0);

    flywheelSim.setInputVoltage(volts);
    flywheelSim.update(0.02);

    inputs.launchConnected = true;
    inputs.launchVelocityRpm = flywheelSim.getAngularVelocityRPM();
    inputs.launchAppliedVolts = volts;
    inputs.launchCurrentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
    inputs.transferConnected = true;
    inputs.transferAppliedVolts = transferVolts;
    inputs.transferCurrentAmps = 0.0;
  }

  @Override
  public void setLaunchVelocity(double rpm) {
    closedLoop = true;
    targetRps = rpm / 60.0;
  }

  @Override
  public void setLaunchPercent(double percent) {
    closedLoop = false;
    openLoopVolts = percent * 12.0;
  }

  @Override
  public void setTransferPercent(double percent) {
    transferVolts = percent * 12.0;
  }

  @Override
  public void setLaunchGains(double kS, double kV, double kP) {
    this.kS = kS;
    this.kV = kV;
    this.kP = kP;
  }

  @Override
  public void stop() {
    closedLoop = false;
    openLoopVolts = 0.0;
    transferVolts = 0.0;
  }
}
