package frc2713.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class CameraMountsTest {
  @ParameterizedTest
  @CsvSource({"front_1, 1, 0, 1", "right_2, 0, -1, -1", "back_3, -1, 0, -1", "left_4, 0, 1, 1"})
  void camerasSitOnTheirNamedSideAndLookOutward(
      String name, double outwardX, double outwardY, double cameraUpSign) {
    var mount =
        VisionConstants.CAMERAS.stream()
            .filter(c -> c.name().equals(name))
            .findFirst()
            .orElseThrow()
            .robotToCamera();
    // Check the physical side independently of the specific mounting measurements.
    double outwardPosition = mount.getX() * outwardX + mount.getY() * outwardY;
    double sidewaysPosition = -mount.getX() * outwardY + mount.getY() * outwardX;
    assertTrue(outwardPosition > Math.abs(sidewaysPosition), name + " must be on its named side");
    assertTrue(mount.getZ() > 0, name + " must be above the floor");

    // PhotonVision looks along camera +X, not the optical frame's +Z.
    var sightline = new Translation3d(1, 0, 0).rotateBy(mount.getRotation());
    assertTrue(sightline.getX() * outwardX + sightline.getY() * outwardY > 0.9);
    assertEquals(0, -sightline.getX() * outwardY + sightline.getY() * outwardX, 1e-9);
    assertTrue(sightline.getZ() > 0, name + " must aim above the horizon");

    var cameraUp = new Translation3d(0, 0, 1).rotateBy(mount.getRotation());
    assertTrue(cameraUp.getZ() * cameraUpSign > 0.9, name + " must retain its mounting roll");
  }
}
