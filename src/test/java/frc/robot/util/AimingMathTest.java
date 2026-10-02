package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/** Unit tests for the aiming math. Run with: ./gradlew test */
class AimingMathTest {
  private static final double EPSILON = 1e-9;

  @Test
  void bearingPointsFromRobotToTarget() {
    var robot = new Translation2d(0.0, 0.0);
    assertEquals(0.0, AimingMath.bearing(robot, new Translation2d(1.0, 0.0)).getDegrees(), EPSILON);
    assertEquals(
        45.0, AimingMath.bearing(robot, new Translation2d(1.0, 1.0)).getDegrees(), EPSILON);
    assertEquals(
        90.0, AimingMath.bearing(robot, new Translation2d(0.0, 1.0)).getDegrees(), EPSILON);
  }

  @Test
  void aimingWithTheBackMeansFaceAway() {
    var robot = new Translation2d(0.0, 0.0);
    var target = new Translation2d(1.0, 0.0);
    // Aim with the front: face the target
    assertEquals(
        0.0, AimingMath.headingToFaceTarget(robot, target, Rotation2d.kZero).getDegrees(), EPSILON);
    // Aim with the back: robot faces directly away so the back points at the target
    assertEquals(
        180.0,
        Math.abs(AimingMath.headingToFaceTarget(robot, target, Rotation2d.k180deg).getDegrees()),
        EPSILON);
  }

  @Test
  void tagToTheRightMeansTurnRight() {
    // Facing 90°, tag 10° to the RIGHT (tx = +10) -> should turn to 80° (clockwise is negative)
    var target = AimingMath.headingFromTx(Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(10));
    assertEquals(80.0, target.getDegrees(), EPSILON);
  }

  @Test
  void bearingRateMatchesNumericalDerivative() {
    var target = new Translation2d(4.6, 4.0);
    var robot = new Translation2d(2.0, 2.5);
    double vx = 1.3;
    double vy = -2.1;
    double dt = 1e-6;

    double analytic = AimingMath.bearingRateRadPerSec(robot, target, vx, vy);
    var robotLater = robot.plus(new Translation2d(vx * dt, vy * dt));
    double numeric =
        AimingMath.bearing(robotLater, target).minus(AimingMath.bearing(robot, target)).getRadians()
            / dt;

    assertEquals(numeric, analytic, 1e-4);
  }

  @Test
  void drivingStraightAtTargetDoesNotChangeBearing() {
    var target = new Translation2d(5.0, 0.0);
    var robot = new Translation2d(0.0, 0.0);
    assertEquals(0.0, AimingMath.bearingRateRadPerSec(robot, target, 2.0, 0.0), EPSILON);
  }
}
