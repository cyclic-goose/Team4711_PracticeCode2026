package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel shooter with a transfer motor.
 *
 * <p>Normal use: {@link #spinUpForDistance} looks up the right RPM for the current distance to the
 * hub in {@link ShooterConstants#DISTANCE_TO_RPM}. To BUILD that table, use {@link #spinUpManual()}
 * and change /Tuning/Shooter/ManualRPM on the dashboard.
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final TunableNumber kS = new TunableNumber("Shooter/kS", ShooterConstants.DEFAULT_KS);
  private final TunableNumber kV = new TunableNumber("Shooter/kV", ShooterConstants.DEFAULT_KV);
  private final TunableNumber kP = new TunableNumber("Shooter/kP", ShooterConstants.DEFAULT_KP);
  private final TunableNumber manualRpm = new TunableNumber("Shooter/ManualRPM", 2500.0);

  private final Alert launchDisconnectedAlert =
      new Alert("Shooter launch motor disconnected (CAN).", AlertType.kError);
  private final Alert transferDisconnectedAlert =
      new Alert("Shooter transfer motor disconnected (CAN).", AlertType.kError);

  private double targetRpm = 0.0;
  private boolean closedLoop = false;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    int id = hashCode();
    // Single "|" on purpose so every value gets checked (and remembered) each loop
    if (kS.hasChanged(id) | kV.hasChanged(id) | kP.hasChanged(id)) {
      io.setLaunchGains(kS.get(), kV.get(), kP.get());
    }

    launchDisconnectedAlert.set(!inputs.launchConnected);
    transferDisconnectedAlert.set(!inputs.transferConnected);
    Logger.recordOutput("Shooter/TargetRPM", closedLoop ? targetRpm : 0.0);
  }

  /** Spin the flywheel at a closed-loop speed. */
  public void setRpm(double rpm) {
    targetRpm = rpm;
    closedLoop = true;
    io.setLaunchVelocity(rpm);
  }

  /** Spin the flywheel open-loop (-1 to 1), like the old code. */
  public void setLaunchPercent(double percent) {
    closedLoop = false;
    io.setLaunchPercent(percent);
  }

  public void setTransferPercent(double percent) {
    io.setTransferPercent(percent);
  }

  public void stop() {
    closedLoop = false;
    targetRpm = 0.0;
    io.stop();
  }

  public double getVelocityRpm() {
    return inputs.launchVelocityRpm;
  }

  /** True when the flywheel is within tolerance of a nonzero closed-loop target. */
  @AutoLogOutput(key = "Shooter/AtSpeed")
  public boolean atSpeed() {
    return closedLoop
        && targetRpm > 0.0
        && Math.abs(inputs.launchVelocityRpm - targetRpm) < ShooterConstants.RPM_TOLERANCE;
  }

  /** Looks up the RPM for a distance in the shooter table. */
  public static double rpmForDistance(double distanceMeters) {
    return ShooterConstants.DISTANCE_TO_RPM.get(distanceMeters);
  }

  /** Holds the flywheel at the table RPM for a changing distance. Stops when the command ends. */
  public Command spinUpForDistance(DoubleSupplier distanceMeters) {
    return run(() -> setRpm(rpmForDistance(distanceMeters.getAsDouble())))
        .finallyDo(this::stop)
        .withName("Shooter.SpinUpForDistance");
  }

  /** Holds the flywheel at /Tuning/Shooter/ManualRPM. Use this to build the shooter table. */
  public Command spinUpManual() {
    return run(() -> setRpm(manualRpm.get())).finallyDo(this::stop).withName("Shooter.Manual");
  }

  /** Runs the transfer (feeds a ball into the flywheel) while held. */
  public Command runTransfer(double percent) {
    return startEnd(() -> setTransferPercent(percent), () -> setTransferPercent(0.0))
        .withName("Shooter.Transfer");
  }
}
