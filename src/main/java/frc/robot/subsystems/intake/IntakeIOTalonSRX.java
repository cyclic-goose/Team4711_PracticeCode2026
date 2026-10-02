package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.DioPorts;

/** Real intake hardware: three TalonSRX (Phoenix 5) and two limit switches. */
public class IntakeIOTalonSRX implements IntakeIO {
  private final WPI_TalonSRX feedMotor1 = new WPI_TalonSRX(CanIds.FEED_MOTOR_1);
  private final WPI_TalonSRX feedMotor2 = new WPI_TalonSRX(CanIds.FEED_MOTOR_2);
  private final WPI_TalonSRX feedMoveMotor = new WPI_TalonSRX(CanIds.FEED_MOVE_MOTOR);

  private final DigitalInput limitSwitchFront = new DigitalInput(DioPorts.INTAKE_LIMIT_FRONT);
  private final DigitalInput limitSwitchBack = new DigitalInput(DioPorts.INTAKE_LIMIT_BACK);

  private double feedPercent = 0.0;
  private double feedMovePercent = 0.0;

  public IntakeIOTalonSRX() {
    for (var motor : new WPI_TalonSRX[] {feedMotor1, feedMotor2, feedMoveMotor}) {
      motor.configFactoryDefault();
      motor.setInverted(false);
      motor.configVoltageCompSaturation(12.0);
      motor.enableVoltageCompensation(true);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // The switches read false when pressed (wired normally-open to ground), so invert them
    inputs.frontLimitPressed = !limitSwitchFront.get();
    inputs.backLimitPressed = !limitSwitchBack.get();
    inputs.feedAppliedPercent = feedPercent;
    inputs.feedMoveAppliedPercent = feedMovePercent;
  }

  @Override
  public void setFeedPercent(double percent) {
    feedPercent = percent;
    feedMotor1.set(ControlMode.PercentOutput, percent);
    feedMotor2.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void setFeedMovePercent(double percent) {
    feedMovePercent = percent;
    feedMoveMotor.set(ControlMode.PercentOutput, percent);
  }
}
