package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.drive.DriveConstants;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

public class SwerveDrive {

  SwervePod[] pods;
  Pigeon2 pigeon;

  SwerveDrivePoseEstimator odometry;

  public SwerveDrive() {
    odometry = new SwerveDrivePoseEstimator(
        new SwerveDriveKinematics(DriveConstants.moduleTranslations),
        pigeon.getRotation2d(),
        getSwervePositions(),
        Pose2d.ZERO
    );

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
    odometry.update(pigeon.getRotation2d(), getSwervePositions());
  }

  public void drive() {
  }

  public SwerveModulePosition[] getSwervePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; ++i) {
      positions[i] = pods[i].getPosition();
    }
    return positions;
  }
}
