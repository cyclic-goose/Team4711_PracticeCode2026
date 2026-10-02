package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CanIds;

/**
 * Real shooter hardware: two TalonFX motor controllers (Phoenix 6).
 *
 * <p>The flywheel uses VelocityVoltage, which runs the speed control loop ON the motor controller
 * at 1000 Hz. The robot code only sends "hold 3000 RPM"; the TalonFX does the rest. That is much
 * more consistent than percent output, which changes with battery voltage and ball contact.
 */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX launchTalon = new TalonFX(CanIds.LAUNCH_MOTOR);
  private final TalonFX transferTalon = new TalonFX(CanIds.TRANSFER_MOTOR);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final DutyCycleOut launchDutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut transferDutyCycle = new DutyCycleOut(0.0);

  private final StatusSignal<AngularVelocity> launchVelocity = launchTalon.getVelocity();
  private final StatusSignal<Voltage> launchAppliedVolts = launchTalon.getMotorVoltage();
  private final StatusSignal<Current> launchCurrent = launchTalon.getStatorCurrent();
  private final StatusSignal<Voltage> transferAppliedVolts = transferTalon.getMotorVoltage();
  private final StatusSignal<Current> transferCurrent = transferTalon.getStatorCurrent();

  public ShooterIOTalonFX() {
    var launchConfig = new TalonFXConfiguration();
    launchConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Let the flywheel coast down
    launchConfig.MotorOutput.Inverted =
        ShooterConstants.LAUNCH_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    launchConfig.CurrentLimits.StatorCurrentLimit = 60.0;
    launchConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // Makes getVelocity() and VelocityVoltage use FLYWHEEL rotations instead of motor rotations
    launchConfig.Feedback.SensorToMechanismRatio = ShooterConstants.LAUNCH_GEAR_RATIO;
    launchConfig.Slot0.kS = ShooterConstants.DEFAULT_KS;
    launchConfig.Slot0.kV = ShooterConstants.DEFAULT_KV;
    launchConfig.Slot0.kP = ShooterConstants.DEFAULT_KP;
    tryUntilOk(5, () -> launchTalon.getConfigurator().apply(launchConfig, 0.25));

    var transferConfig = new TalonFXConfiguration();
    transferConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    transferConfig.MotorOutput.Inverted =
        ShooterConstants.TRANSFER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    transferConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    transferConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> transferTalon.getConfigurator().apply(transferConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        launchVelocity,
        launchAppliedVolts,
        launchCurrent,
        transferAppliedVolts,
        transferCurrent);
    ParentDevice.optimizeBusUtilizationForAll(launchTalon, transferTalon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var launchStatus =
        BaseStatusSignal.refreshAll(launchVelocity, launchAppliedVolts, launchCurrent);
    var transferStatus = BaseStatusSignal.refreshAll(transferAppliedVolts, transferCurrent);

    inputs.launchConnected = launchStatus.isOK();
    inputs.launchVelocityRpm = launchVelocity.getValueAsDouble() * 60.0;
    inputs.launchAppliedVolts = launchAppliedVolts.getValueAsDouble();
    inputs.launchCurrentAmps = launchCurrent.getValueAsDouble();

    inputs.transferConnected = transferStatus.isOK();
    inputs.transferAppliedVolts = transferAppliedVolts.getValueAsDouble();
    inputs.transferCurrentAmps = transferCurrent.getValueAsDouble();
  }

  @Override
  public void setLaunchVelocity(double rpm) {
    launchTalon.setControl(velocityRequest.withVelocity(rpm / 60.0)); // Phoenix uses rot/sec
  }

  @Override
  public void setLaunchPercent(double percent) {
    launchTalon.setControl(launchDutyCycle.withOutput(percent));
  }

  @Override
  public void setTransferPercent(double percent) {
    transferTalon.setControl(transferDutyCycle.withOutput(percent));
  }

  @Override
  public void setLaunchGains(double kS, double kV, double kP) {
    var slot0 = new Slot0Configs().withKS(kS).withKV(kV).withKP(kP);
    tryUntilOk(5, () -> launchTalon.getConfigurator().apply(slot0, 0.25));
  }

  @Override
  public void stop() {
    launchTalon.stopMotor();
    transferTalon.stopMotor();
  }
}
