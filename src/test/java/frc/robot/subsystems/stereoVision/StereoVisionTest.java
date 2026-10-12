package frc.robot.subsystems.stereoVision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class StereoVisionTest {
  @BeforeAll
  static void initialize() {
    assertTrue(HAL.initialize(500, 0));
  }

  private static class Camera implements StereoVisionIO {
    final Transform3d pose;
    final Translation3d target;
    boolean connected = true;
    double age = 0;
    boolean seesFuel = true;

    Camera(Transform3d pose, Translation3d target) {
      this.pose = pose;
      this.target = target;
    }

    @Override
    public Transform3d getRobotToCamera() {
      return pose;
    }

    @Override
    public void updateInputs(StereoVisionIOInputs inputs) {
      var local = target.minus(pose.getTranslation()).rotateBy(pose.getRotation().unaryMinus());
      inputs.connected = connected;
      inputs.timestampSeconds = Timer.getFPGATimestamp() - age;
      inputs.targetClassIds = seesFuel ? new int[] {0} : new int[0];
      // PhotonVision v2026: yaw positive right, pitch positive up (elevation).
      inputs.targetYawDegrees =
          new double[] {Math.toDegrees(Math.atan2(-local.getY(), local.getX()))};
      inputs.targetPitchDegrees =
          new double[] {
            Math.toDegrees(Math.atan2(local.getZ(), Math.hypot(local.getX(), local.getY())))
          };
    }
  }

  @Test
  void reconstructsOffCenterBallBelowParallelCameras() {
    checkPosition(new Rotation3d(), new Rotation3d());
  }

  @Test
  void appliesMountRotationAfterCameraRelativeRay() {
    checkPosition(new Rotation3d(0.05, 0.12, 0.2), new Rotation3d(-0.04, 0.08, -0.1));
  }

  private void checkPosition(Rotation3d leftRotation, Rotation3d rightRotation) {
    var target = new Translation3d(1.8, 0.22, -0.5);
    var left = new Camera(new Transform3d(new Translation3d(0, 0.15, 0), leftRotation), target);
    var right = new Camera(new Transform3d(new Translation3d(0, -0.15, 0), rightRotation), target);
    var vision = new StereoVision(left, right);
    vision.refreshInputs();
    var positions = vision.getFuelPositionsRobotRelative();
    assertEquals(1, positions.size());
    assertEquals(0.0, positions.get(0).getDistance(target), 1e-8);
    left.connected = false;
    vision.refreshInputs();
    assertTrue(vision.getFuelPositionsRobotRelative().isEmpty());
    left.connected = true;
    left.age = 1.0;
    vision.refreshInputs();
    assertTrue(vision.getFuelPositionsRobotRelative().isEmpty());
    left.age = 0;
    right.seesFuel = false;
    vision.refreshInputs();
    assertTrue(vision.getFuelPositionsRobotRelative().isEmpty());
  }
}
