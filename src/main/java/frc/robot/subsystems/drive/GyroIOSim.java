package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private GyroSimulation gyroIOSim;

  public GyroIOSim() {
    gyroIOSim = DriveConstants.swerveDriveSimulation.getGyroSimulation();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = new Rotation2d(gyroIOSim.getGyroReading().getRadians());
    inputs.yawVelocityRadPerSec = gyroIOSim.getMeasuredAngularVelocity().magnitude();
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    // Run closed-loop control

  }
}
