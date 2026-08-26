package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants.Hub;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class BallSimulatorTest {
  private BallSimulator simulator;

  @BeforeEach
  public void setUp() {
    simulator = BallSimulator.getInstance();
    simulator.setBallsInRobot(0);
    simulator.resetBallsScored();
  }

  @Test
  public void testBallInventoryManagement() {
    assertEquals(0, simulator.getBallsInRobot());
    assertFalse(simulator.hasBalls());

    simulator.addBall();
    simulator.addBall();
    assertEquals(2, simulator.getBallsInRobot());
    assertTrue(simulator.hasBalls());

    simulator.removeBall();
    assertEquals(1, simulator.getBallsInRobot());

    // Fill to capacity
    simulator.setBallsInRobot(BallSimulator.MAX_CAPACITY + 5);
    assertEquals(BallSimulator.MAX_CAPACITY, simulator.getBallsInRobot());
    assertTrue(simulator.isFull());

    // Remove below zero
    simulator.setBallsInRobot(-3);
    assertEquals(0, simulator.getBallsInRobot());
  }

  @Test
  public void testTrajectoryPredictionAccurateHit() {
    // Position robot facing Blue Hub so that shooter is exactly 2.0m away
    Translation2d blueHub = Hub.blueCenter;
    // Shooter is offset by (-0.163, 0) relative to robot. When robot is facing 180 degrees,
    // shooter field offset is (+0.163, 0).
    // So robot at (blueHub.X + 2.0 - 0.163, blueHub.Y) results in shooter at (blueHub.X + 2.0,
    // blueHub.Y).
    Pose2d robotPose =
        new Pose2d(blueHub.getX() + 2.0 - 0.163, blueHub.getY(), Rotation2d.fromDegrees(180.0));

    // Distance is 2.0m. From formula calculateSetpoint: RPM ≈ 223 * 2.0 + 751 ≈ 1197 RPM
    BallSimulator.TrajectoryPrediction prediction = simulator.predictTrajectory(robotPose, 1200.0);

    assertEquals(2.0, prediction.distanceToHub(), 0.05);
    assertTrue(prediction.willHit(), "Shot should hit when aimed directly at hub with correct RPM");
    assertEquals(0.0, Math.toDegrees(prediction.angleErrorRad()), 0.5);
  }

  @Test
  public void testTrajectoryPredictionMisalignedMisses() {
    // Position robot 2.0m away from Blue Hub but turned 90 degrees away
    Translation2d blueHub = Hub.blueCenter;
    Pose2d robotPose =
        new Pose2d(blueHub.getX() + 2.0 - 0.163, blueHub.getY(), Rotation2d.fromDegrees(90.0));

    BallSimulator.TrajectoryPrediction prediction = simulator.predictTrajectory(robotPose, 1200.0);

    assertFalse(
        prediction.willHit(), "Shot should miss when robot is turned 90 degrees away from Hub");
    assertTrue(prediction.angleErrorRad() > 0.5);
  }

  @Test
  public void testTrajectoryPredictionUnderpoweredMisses() {
    // Position robot 4.0m away from Blue Hub but with only 500 RPM (very weak)
    Translation2d blueHub = Hub.blueCenter;
    Pose2d robotPose =
        new Pose2d(blueHub.getX() + 4.0 - 0.163, blueHub.getY(), Rotation2d.fromDegrees(180.0));

    BallSimulator.TrajectoryPrediction prediction = simulator.predictTrajectory(robotPose, 500.0);

    assertFalse(prediction.willHit(), "Underpowered shot should fall short of Hub");
  }
}
