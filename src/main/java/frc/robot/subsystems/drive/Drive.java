package frc.robot.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.gyro3d.GyroIO;
import frc.lib.io.gyro3d.GyroIOInputsAutoLogged;
import frc.lib.utils.RobotOdometry;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SwerveDriveKinematics kinematics = RobotConstants.get().kinematics();
  private final SwerveDrivePoseEstimator odometry;
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final double MAX_LINEAR_SPEED = RobotConstants.get().maxLinearSpeed();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    odometry = RobotOdometry.getInstance().getPoseEstimator();
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelPositions[i] = modules[i].getPosition();
    }
    if (gyroInputs.connected) {
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        wheelDeltas[i] = modules[i].getPositionDelta();
      }
      // The twist represents the motion of the robot since the last
      // loop cycle in x, y, and theta based only on the modules,
      // without the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      odometry.update(lastGyroRotation.plus(new Rotation2d(twist.dtheta)), wheelPositions);
      lastGyroRotation = lastGyroRotation.plus(new Rotation2d(twist.dtheta));
    } else {
      lastGyroRotation = gyroInputs.yaw;
      odometry.update(gyroInputs.yaw, wheelPositions);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = RobotConstants.get().moduleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return odometry.getEstimatedPosition().getRotation();
  }

  public double getPitchVelocity() {
    return gyroInputs.pitchRatePerSec;
  }

  public double getRollVelocity() {
    return gyroInputs.rollRatePerSec;
  }

  public Rotation3d getRotation3d() {
    return gyroInputs.rotation3d;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelPositions[i] = modules[i].getPosition();
    }
    odometry.resetPosition(gyroInputs.yaw, wheelPositions, pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /**
   * Checks if the robot is upright within a certain threshold (checks if it will be considered
   * balanced by the charge station)
   *
   * @return true if the robot is upright within the threshold, false otherwise
   */
  public boolean isUpright() {
    double pitch = Units.radiansToDegrees(getRotation3d().getY());
    double roll = Units.radiansToDegrees(getRotation3d().getX());

    return Math.abs(roll) < 2.3 && Math.abs(pitch) < 2.3;
  }
}
