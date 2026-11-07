package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwervePod.ID.BACK_LEFT;
import static frc.robot.subsystems.swerve.SwervePod.ID.BACK_RIGHT;
import static frc.robot.subsystems.swerve.SwervePod.ID.FRONT_LEFT;
import static frc.robot.subsystems.swerve.SwervePod.ID.FRONT_RIGHT;

import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;
import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Logged;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;


@Logged
public class SwerveDrive<DriveMotorT, TurnMotorT> {

  /**
   * 1. Idle
   * <p>
   * 2. Joystick Drive
   * <p>
   * 3. Drive To Pose
   * <p>
   * 4. Rotate to Orientation
   * <p>
   * 5. Path Following
   */

  SwervePod[] pods;
  Gyro gyro;

  SwerveDriveKinematics kinematics;
  SwerveDrivePoseEstimator odometry;

  public SwerveDrive(Supplier<DriveMotorT> driveMotorSupplier,
      Supplier<TurnMotorT> turnMotorSupplier, Gyro gyro) {
    this.gyro = gyro;

    pods = new SwervePod[4];
    pods[0] = new SwervePod<>(driveMotorSupplier.get(), turnMotorSupplier.get(), FRONT_LEFT);
    pods[1] = new SwervePod<>(driveMotorSupplier.get(), turnMotorSupplier.get(), FRONT_RIGHT);
    pods[2] = new SwervePod<>(driveMotorSupplier.get(), turnMotorSupplier.get(), BACK_LEFT);
    pods[3] = new SwervePod<>(driveMotorSupplier.get(), turnMotorSupplier.get(), BACK_RIGHT);

    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
    odometry = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        getSwervePositions(),
        Pose2d.ZERO
    );

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
    odometry.update(gyro.getRotation2d(), getSwervePositions());
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
