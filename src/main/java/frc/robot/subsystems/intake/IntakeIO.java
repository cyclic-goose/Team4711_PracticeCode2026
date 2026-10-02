package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the intake (rollers + deploy motor + two limit switches). */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean frontLimitPressed = false;
    public boolean backLimitPressed = false;
    public double feedAppliedPercent = 0.0;
    public double feedMoveAppliedPercent = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Both intake rollers, -1 to 1. */
  public default void setFeedPercent(double percent) {}

  /** Deploy/retract motor, -1 to 1 (negative = deploy/forward, positive = retract/back). */
  public default void setFeedMovePercent(double percent) {}
}
