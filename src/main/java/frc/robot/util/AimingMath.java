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
   * The robot heading that makes the SHOOTER point at the target.
   *
   * @param robot Robot position on the field
   * @param target Target position on the field
   * @param shooterFacing Which way the shooter points relative to the robot's front (0° = shooter
   *     on the front, 180° = shooter on the back)
   */
  public static Rotation2d headingToFaceTarget(
      Translation2d robot, Translation2d target, Rotation2d shooterFacing) {
    return bearing(robot, target).minus(shooterFacing);
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
