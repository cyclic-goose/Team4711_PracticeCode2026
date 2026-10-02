package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Set;

/**
 * Everything the code knows about the field.
 *
 * <p>All field coordinates in this project use the WPILib "always blue origin" convention: (0, 0)
 * is the corner of the field on the BLUE alliance wall, +X points from the blue wall toward the red
 * wall, +Y points to the left when standing behind the blue driver station, and 0° heading means
 * facing +X (away from the blue driver station). Red-alliance positions are found by "flipping"
 * blue positions, never by using a different origin.
 *
 * <p>The 2026 (REBUILT) field is rotationally symmetric: a red position is the blue position
 * rotated 180° around the center of the field. PathPlanner's FlippingUtil uses the same rule by
 * default, so paths drawn for blue automatically work for red.
 */
public final class FieldConstants {
  private FieldConstants() {}

  /** Official 2026 AprilTag positions (the "welded" field, which most events used). */
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double fieldLength = aprilTagLayout.getFieldLength(); // meters (X)
  public static final double fieldWidth = aprilTagLayout.getFieldWidth(); // meters (Y)

  /** The 8 tags mounted on each hub (2 per face). */
  public static final Set<Integer> blueHubTagIds = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

  public static final Set<Integer> redHubTagIds = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

  /**
   * Hub centers, computed from the tag layout instead of typed in by hand. The tags sit on the 4
   * faces of the hub, so the middle of the box they outline is the middle of the hub. (Works out to
   * about (4.63, 4.03) for blue and (11.92, 4.03) for red.)
   *
   * <p>Note: simply AVERAGING the 8 tag positions would be ~9 cm off, because each face has one
   * centered tag and one offset tag, and the offsets all go the same way around the hub.
   */
  public static final Translation2d blueHubCenter = centerOfTags(blueHubTagIds);

  public static final Translation2d redHubCenter = centerOfTags(redHubTagIds);

  /**
   * An example spot to shoot from, written for the BLUE alliance: 2.5 m straight back from the
   * center of the blue hub. Use {@link #flipIfRed(Pose2d)} or PathPlanner's pathfindToPoseFlipped()
   * to get the red version. Move it wherever your shooter is most accurate.
   */
  public static final Translation2d blueShootingPosition =
      new Translation2d(blueHubCenter.getX() - 2.5, blueHubCenter.getY());

  /** True when the Driver Station says we are on the red alliance (defaults to blue). */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /** The center of OUR hub (the one we score in), based on the current alliance. */
  public static Translation2d getOurHubCenter() {
    return isRedAlliance() ? redHubCenter : blueHubCenter;
  }

  /** True if the tag ID is on either hub. */
  public static boolean isHubTag(int tagId) {
    return blueHubTagIds.contains(tagId) || redHubTagIds.contains(tagId);
  }

  /** Rotates a blue-alliance pose 180° around the field center when we are red. */
  public static Pose2d flipIfRed(Pose2d bluePose) {
    if (!isRedAlliance()) {
      return bluePose;
    }
    return new Pose2d(
        fieldLength - bluePose.getX(),
        fieldWidth - bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  /** Center of the rectangle that contains all the given tags (on the field floor). */
  private static Translation2d centerOfTags(Set<Integer> tagIds) {
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (int id : tagIds) {
      var pose =
          aprilTagLayout
              .getTagPose(id)
              .orElseThrow(() -> new IllegalStateException("Tag " + id + " missing from layout"));
      minX = Math.min(minX, pose.getX());
      maxX = Math.max(maxX, pose.getX());
      minY = Math.min(minY, pose.getY());
      maxY = Math.max(maxY, pose.getY());
    }
    return new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0);
  }
}
