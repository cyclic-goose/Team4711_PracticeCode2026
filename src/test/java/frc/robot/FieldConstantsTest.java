package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.util.FlippingUtil;
import org.junit.jupiter.api.Test;

/** Sanity checks on the field geometry the aiming code depends on. */
class FieldConstantsTest {
  @Test
  void hubCentersAreWhereTheFieldDrawingsSayTheyAre() {
    // Each hub is centered across the field (Y) and ~4.63 m from its own alliance wall (X)
    assertEquals(4.6255, FieldConstants.blueHubCenter.getX(), 0.01);
    assertEquals(FieldConstants.fieldWidth / 2.0, FieldConstants.blueHubCenter.getY(), 0.01);
    assertEquals(FieldConstants.fieldLength - 4.6255, FieldConstants.redHubCenter.getX(), 0.01);
    assertEquals(FieldConstants.fieldWidth / 2.0, FieldConstants.redHubCenter.getY(), 0.01);
  }

  @Test
  void pathPlannerFlipsTheBlueHubOntoTheRedHub() {
    // If this fails, PathPlanner's flipping doesn't match the field and red autos would be wrong
    var flipped = FlippingUtil.flipFieldPosition(FieldConstants.blueHubCenter);
    assertEquals(0.0, flipped.getDistance(FieldConstants.redHubCenter), 0.05);
  }

  @Test
  void hubTagSetsDoNotOverlap() {
    for (int id : FieldConstants.blueHubTagIds) {
      assertTrue(!FieldConstants.redHubTagIds.contains(id));
      assertTrue(FieldConstants.isHubTag(id));
    }
  }
}
