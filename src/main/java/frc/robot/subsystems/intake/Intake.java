package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Intake with a deploy/retract motor and rollers. Same behavior as the old code: bumpers move the
 * intake, triggers run the rollers, and "locked" retracts it and ignores the controls.
 */
public class Intake extends SubsystemBase {
  // Speeds carried over from the old code
  private static final double DEPLOY_PERCENT = -0.90;
  private static final double RETRACT_PERCENT = 0.95;
  private static final double FEED_PERCENT = 0.65;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean locked = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @AutoLogOutput(key = "Intake/Locked")
  public boolean isLocked() {
    return locked;
  }

  public void toggleLocked() {
    locked = !locked;
  }

  public void setFeedPercent(double percent) {
    io.setFeedPercent(percent);
  }

  public void setFeedMovePercent(double percent) {
    io.setFeedMovePercent(percent);
  }

  public void stop() {
    io.setFeedPercent(0.0);
    io.setFeedMovePercent(0.0);
  }

  /**
   * Driver control, ported from the old RobotContainer default command.
   *
   * @param deployButton Moves the intake out (stops at the front limit switch)
   * @param retractButton Moves the intake in (stops at the back limit switch)
   * @param feedForward Trigger value (0-1) to run the rollers inward
   * @param feedReverse Trigger value (0-1) to run the rollers outward
   */
  public Command manualControl(
      BooleanSupplier deployButton,
      BooleanSupplier retractButton,
      DoubleSupplier feedForward,
      DoubleSupplier feedReverse) {
    return run(() -> {
          if (locked) {
            // Retract until the back limit switch is pressed, and ignore everything else
            io.setFeedMovePercent(inputs.backLimitPressed ? 0.0 : RETRACT_PERCENT);
            io.setFeedPercent(0.0);
            return;
          }

          if (deployButton.getAsBoolean() && !inputs.frontLimitPressed) {
            io.setFeedMovePercent(DEPLOY_PERCENT);
          } else if (retractButton.getAsBoolean() && !inputs.backLimitPressed) {
            io.setFeedMovePercent(RETRACT_PERCENT);
          } else {
            io.setFeedMovePercent(0.0);
          }

          // Rollers only run when the intake is NOT fully retracted (same as the old code)
          if (feedForward.getAsDouble() > 0.1 && !inputs.backLimitPressed) {
            io.setFeedPercent(FEED_PERCENT);
          } else if (feedReverse.getAsDouble() > 0.1 && !inputs.backLimitPressed) {
            io.setFeedPercent(-FEED_PERCENT);
          } else {
            io.setFeedPercent(0.0);
          }
        })
        .withName("Intake.ManualControl");
  }
}
