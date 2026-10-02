package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.AimController;
import frc.robot.util.AimingMath;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestFactory;

/**
 * Checks EVERY path and auto in src/main/deploy/pathplanner automatically. When you draw a new path
 * or auto in the PathPlanner GUI, just run ./gradlew test and it gets checked here too:
 *
 * <ul>
 *   <li>Every path: the simulated robot follows it and must finish where the path ends (within 15
 *       cm and 5°). Catches impossible paths (too tight, too fast for the robot config).
 *   <li>Every auto: every path it uses must exist, and every named command it uses must be
 *       registered in RobotContainer.registerNamedCommands() (catches typos, which PathPlanner
 *       otherwise silently skips).
 * </ul>
 */
class AllPathsAndAutosTest {
  private static final File PATHPLANNER_DIR = new File("src/main/deploy/pathplanner");

  @BeforeAll
  static void setUp() {
    SimTestHelper.robot();
  }

  @AfterEach
  void reset() {
    SimTestHelper.reset();
  }

  private static List<String> namesIn(String folder, String extension) {
    File[] files = new File(PATHPLANNER_DIR, folder).listFiles((dir, n) -> n.endsWith(extension));
    List<String> names = new ArrayList<>();
    if (files != null) {
      Arrays.sort(files);
      for (File file : files) {
        names.add(file.getName().substring(0, file.getName().length() - extension.length()));
      }
    }
    return names;
  }

  @TestFactory
  Stream<DynamicTest> everyPathEndsWhereItShould() {
    return namesIn("paths", ".path").stream()
        .map(
            name ->
                DynamicTest.dynamicTest(
                    "path: " + name,
                    () -> {
                      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
                      var drive = SimTestHelper.robot().getDrive();
                      SimTestHelper.teleport(path.getStartingHolonomicPose().orElseThrow());

                      SimTestHelper.runUntilFinished(AutoBuilder.followPath(path), 20.0);

                      var points = path.getAllPathPoints();
                      Translation2d expectedEnd = points.get(points.size() - 1).position;
                      Pose2d actual = drive.getPose();
                      assertEquals(
                          0.0,
                          actual.getTranslation().getDistance(expectedEnd),
                          0.15,
                          name + " ended at " + actual + ", expected " + expectedEnd);
                      assertEquals(
                          0.0,
                          path.getGoalEndState()
                              .rotation()
                              .minus(actual.getRotation())
                              .getDegrees(),
                          5.0,
                          name + " ended facing the wrong way");
                    }));
  }

  @TestFactory
  Stream<DynamicTest> everyAutoUsesRealPathsAndRegisteredCommands() {
    return namesIn("autos", ".auto").stream()
        .map(
            name ->
                DynamicTest.dynamicTest(
                    "auto: " + name,
                    () -> {
                      JsonNode root =
                          new ObjectMapper()
                              .readTree(new File(PATHPLANNER_DIR, "autos/" + name + ".auto"));
                      List<String> problems = new ArrayList<>();
                      checkCommand(root.get("command"), problems);
                      if (!problems.isEmpty()) {
                        fail("Auto '" + name + "': " + String.join("; ", problems));
                      }
                      // Also make sure PathPlanner itself can build it
                      new PathPlannerAuto(name);
                    }));
  }

  /** Walks the auto's command tree looking for missing paths or unregistered named commands. */
  private static void checkCommand(JsonNode command, List<String> problems) {
    String type = command.get("type").asText();
    JsonNode data = command.get("data");
    switch (type) {
      case "path" -> {
        String pathName = data.get("pathName").asText();
        if (!new File(PATHPLANNER_DIR, "paths/" + pathName + ".path").exists()) {
          problems.add("uses path '" + pathName + "' which doesn't exist");
        }
      }
      case "named" -> {
        String commandName = data.get("name").asText();
        if (!NamedCommands.hasCommand(commandName)) {
          problems.add(
              "uses named command '"
                  + commandName
                  + "' which isn't registered in RobotContainer.registerNamedCommands()");
        }
      }
      case "sequential", "parallel", "race", "deadline" -> {
        for (JsonNode child : data.get("commands")) {
          checkCommand(child, problems);
        }
      }
      default -> {} // "wait" and anything else have nothing to check
    }
  }

  @Test
  void classroomTourEndsBackAtTheStartFacingTheHub() {
    var drive = SimTestHelper.robot().getDrive();
    SimTestHelper.teleport(FieldConstants.blueClassroomStart);

    SimTestHelper.runUntilFinished(new PathPlannerAuto("Classroom - Tour"), 30.0);

    Pose2d end = drive.getPose();
    assertEquals(
        0.0,
        end.getTranslation().getDistance(FieldConstants.blueClassroomStart.getTranslation()),
        0.15,
        "Tour ended at " + end);
    var wanted =
        AimingMath.headingToFaceTarget(
            end.getTranslation(), FieldConstants.blueHubCenter, AimController.AIM_SIDE);
    assertEquals(0.0, wanted.minus(end.getRotation()).getDegrees(), 3.0);
  }

  @Test
  void classroomLoopIsFlippedCorrectlyOnRed() {
    // In the classroom the Driver Station is on Red (tags 10 and 9). PathPlanner should rotate the
    // blue path 180° around the field center, so the loop happens in front of the RED hub.
    try {
      SimTestHelper.setAlliance(edu.wpi.first.hal.AllianceStationID.Red1);
      var drive = SimTestHelper.robot().getDrive();
      Pose2d redStart = FieldConstants.flipIfRed(FieldConstants.blueClassroomStart);
      SimTestHelper.teleport(redStart);

      SimTestHelper.runUntilFinished(new PathPlannerAuto("Classroom - Loop"), 20.0);

      Pose2d end = drive.getPose();
      assertTrue(
          end.getX() > FieldConstants.fieldLength / 2.0, "Should be on the red half: " + end);
      assertEquals(0.0, end.getTranslation().getDistance(redStart.getTranslation()), 0.15);
      assertEquals(0.0, redStart.getRotation().minus(end.getRotation()).getDegrees(), 5.0);
    } finally {
      SimTestHelper.setAlliance(edu.wpi.first.hal.AllianceStationID.Blue1);
    }
  }

  @Test
  void autoChooserFoundTheExampleAutos() {
    var names = AutoBuilder.getAllAutoNames();
    assertTrue(names.contains("Classroom - Tour"), "Autos found: " + names);
    assertTrue(names.contains("Example - Back Up And Aim"), "Autos found: " + names);
  }
}
