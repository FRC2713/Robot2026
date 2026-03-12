package frc2713.lib.io;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

class AdvantageScopePathBuilderTest {

  @Nested
  class MakePath {

    @Test
    void emptySegments_returnsBasePath() {
      var pb = new AdvantageScopePathBuilder("Drive");
      assertEquals("Drive", pb.makePath());
    }

    @Test
    void singleSegment_returnsBasePathSlashSegment() {
      var pb = new AdvantageScopePathBuilder("Drive");
      assertEquals("Drive/SwerveStates", pb.makePath("SwerveStates"));
    }

    @Test
    void multipleSegments_returnsFullPath() {
      var pb = new AdvantageScopePathBuilder("Drive");
      assertEquals("Drive/SwerveStates/Setpoints", pb.makePath("SwerveStates", "Setpoints"));
    }

    @Test
    void threeSegments_returnsFullPath() {
      var pb = new AdvantageScopePathBuilder("Turret");
      assertEquals(
          "Turret/API/setPositionSetpointImpl/units",
          pb.makePath("API", "setPositionSetpointImpl", "units"));
    }

    @Test
    void emptyBasePath_stillBuildsCorrectly() {
      var pb = new AdvantageScopePathBuilder("");
      assertEquals("SwerveStates", pb.makePath("SwerveStates"));
      assertEquals("a/b/c", pb.makePath("a", "b", "c"));
    }

    @Test
    void repeatedCalls_returnSameInstance() {
      var pb = new AdvantageScopePathBuilder("Drive");
      String first = pb.makePath("SwerveStates", "Setpoints");
      String second = pb.makePath("SwerveStates", "Setpoints");
      assertSame(first, second, "Cached paths should return same String instance");
    }

    @Test
    void differentPaths_returnDifferentStrings() {
      var pb = new AdvantageScopePathBuilder("Drive");
      String path1 = pb.makePath("SwerveStates", "Setpoints");
      String path2 = pb.makePath("SwerveChassisSpeeds", "Setpoints");
      assertEquals("Drive/SwerveStates/Setpoints", path1);
      assertEquals("Drive/SwerveChassisSpeeds/Setpoints", path2);
    }

    @Test
    void differentBuilders_doNotShareCache() {
      var pb1 = new AdvantageScopePathBuilder("Drive");
      var pb2 = new AdvantageScopePathBuilder("Turret");
      assertEquals("Drive/setpoint", pb1.makePath("setpoint"));
      assertEquals("Turret/setpoint", pb2.makePath("setpoint"));
    }
  }

  @Nested
  class MakeName {

    @Test
    void returnsBasePathSpaceName() {
      var pb = new AdvantageScopePathBuilder("Flywheels");
      assertEquals("Flywheels DefaultCommand", pb.makeName("DefaultCommand"));
    }

    @Test
    void emptyBasePath_returnsSpacePlusName() {
      var pb = new AdvantageScopePathBuilder("");
      assertEquals(" SomeName", pb.makeName("SomeName"));
    }
  }
}
