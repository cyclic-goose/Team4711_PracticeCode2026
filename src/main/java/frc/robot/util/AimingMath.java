package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pure math for aiming, kept separate from commands so it can be unit tested on a laptop (see
 * src/test/java/frc/robot/util/AimingMathTest.java).
 *
 * <p>Angles follow WPILib conventions: counter-clockwise (turning LEFT) is positive.
 */
public final class AimingMath {
  private AimingMath() {}

  /**
   * Field direction from one point to another. Example: from (0,0) to (1,1) is 45°.
   *
   * @param from Usually the robot's position
   * @param to Usually the target's position
   */
  public static Rotation2d bearing(Translation2d from, Translation2d to) {
    return to.minus(from).getAngle();
  }

  /**
   * The robot heading that makes a chosen side of the robot point at the target.
   *
   * @param robot Robot position on the field
   * @param target Target position on the field
   * @param aimSide Which side of the robot should face the target (0° = front, 180° = back)
   */
  public static Rotation2d headingToFaceTarget(
      Translation2d robot, Translation2d target, Rotation2d aimSide) {
    return bearing(robot, target).minus(aimSide);
  }

  /**
   * How fast the direction to the target is changing because the robot is driving (rad/s). If the
   * robot strafes sideways past the hub, the hub's bearing sweeps around even though the robot
   * isn't turning. Feeding this rate forward lets the aim keep up instead of lagging behind.
   *
   * <p>Derivation: bearing = atan2(dy, dx) where (dx, dy) = target - robot. Taking the time
   * derivative with the target standing still gives (dy * vx - dx * vy) / (dx² + dy²).
   *
   * @param robot Robot position (m)
   * @param target Target position (m)
   * @param fieldVx Robot velocity along field X (m/s)
   * @param fieldVy Robot velocity along field Y (m/s)
   */
  public static double bearingRateRadPerSec(
      Translation2d robot, Translation2d target, double fieldVx, double fieldVy) {
    double dx = target.getX() - robot.getX();
    double dy = target.getY() - robot.getY();
    double distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < 1e-6) {
      return 0.0; // Basically on top of the target; avoid dividing by ~0
    }
    return (dy * fieldVx - dx * fieldVy) / distanceSquared;
  }

  /**
   * Converts a Limelight tx reading into a field heading to turn to.
   *
   * <p>tx is positive when the tag is to the RIGHT of the crosshair. Turning right is a NEGATIVE
   * (clockwise) rotation in WPILib, so the heading that centers the tag is (heading when the image
   * was taken) MINUS tx. Using the heading at capture time (instead of right now) cancels out the
   * camera's latency.
   */
  public static Rotation2d headingFromTx(Rotation2d headingAtCapture, Rotation2d tx) {
    return headingAtCapture.minus(tx);
  }
}
