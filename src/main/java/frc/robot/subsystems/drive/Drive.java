package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.gyro2d.Gyro2DIO;
import frc.lib.io.gyro2d.Gyro2DIOInputsAutoLogged;
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private static final SimpleMotorFeedforward driveFF =
      RobotConstants.get().moduleDriveFF().getFeedforward(); // Faster to type and shorter to read
  private static final SimpleMotorFeedforward turnFF =
      RobotConstants.get().moduleTurnFF().getFeedforward();
  private static final ProfiledPIDController[] turnFB = new ProfiledPIDController[4];
  private final ModuleIO[] moduleIOs = new ModuleIO[4];
  private final ModuleIOInputsAutoLogged[] moduleIOInputs =
      new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
      };
  private final Gyro2DIO gyroIO;
  private final Gyro2DIOInputsAutoLogged gyroIOInputs = new Gyro2DIOInputsAutoLogged();

  private double angle = 0;
  private DriveMode driveMode = DriveMode.NORMAL;
  private double characterizationVoltage = 0.0;

  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private final SwerveDriveOdometry odometry;

  private boolean brakeMode = false;

  /**
   * Configures the drive subsystem
   *
   * @param gyroIO Gyro IO
   * @param flIO Front Left Module IO
   * @param frIO Front Right Module IO
   * @param blIO Back Left Module IO
   * @param brIO Back Right Module IO
   */
  public Drive(Gyro2DIO gyroIO, ModuleIO flIO, ModuleIO frIO, ModuleIO blIO, ModuleIO brIO) {
    super();
    this.gyroIO = gyroIO;
    moduleIOs[0] = flIO;
    moduleIOs[1] = frIO;
    moduleIOs[2] = blIO;
    moduleIOs[3] = brIO;

    for (int i = 0; i < 4; i++) {
      this.moduleIOs[i].updateInputs(this.moduleIOInputs[i]);
      turnFB[i] =
          RobotConstants.get()
              .moduleTurnFB()
              .getProfiledPIDController(new TrapezoidProfile.Constraints(550.6, 7585));
      turnFB[i].enableContinuousInput(-Math.PI, Math.PI);
      turnFB[i].reset(this.moduleIOInputs[i].turnPositionAbsolute);
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] =
          new SwerveModulePosition(
              moduleIOInputs[i].drivePosition * (RobotConstants.get().moduleWheelDiameter() / 2),
              new Rotation2d(moduleIOInputs[i].turnPositionAbsolute));
    }

    if (gyroIOInputs.connected) {
      odometry =
          new SwerveDriveOdometry(
              RobotConstants.get().kinematics(),
              Rotation2d.fromDegrees(gyroIOInputs.angle),
              modulePositions);
    } else {
      odometry =
          new SwerveDriveOdometry(
              RobotConstants.get().kinematics(), new Rotation2d(angle), modulePositions);
    }
  }

  @Override
  public void periodic() {

    // Update inputs for IOs
    gyroIO.updateInputs(gyroIOInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroIOInputs);
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleIOInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + i, moduleIOInputs[i]);
    }

    Logger.getInstance()
        .recordOutput(
            "Chassis velocity",
            new double[] {
              setpoint.vxMetersPerSecond, setpoint.vyMetersPerSecond, setpoint.omegaRadiansPerSecond
            });

    // Update angle measurements
    Rotation2d[] turnPositions = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      turnPositions[i] = new Rotation2d(moduleIOInputs[i].turnPositionAbsolute);
    }

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    } else {
      switch (driveMode) {
        case NORMAL:
          // In normal mode, run the controllers for turning and driving based on the current
          // setpoint
          SwerveModuleState[] setpointStates =
              RobotConstants.get().kinematics().toSwerveModuleStates(setpoint);
          SwerveDriveKinematics.desaturateWheelSpeeds(
              setpointStates, RobotConstants.get().maxLinearSpeed());

          // If stationary, go to last state
          boolean isStationary =
              Math.abs(setpoint.vxMetersPerSecond) < 1e-3
                  && Math.abs(setpoint.vyMetersPerSecond) < 1e-3
                  && Math.abs(setpoint.omegaRadiansPerSecond) < 1e-3;

          SwerveModuleState[] setpointStatesOptimized =
              new SwerveModuleState[] {null, null, null, null};
          for (int i = 0; i < 4; i++) {
            // Run turn controller
            setpointStatesOptimized[i] =
                SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
            if (isStationary) {
              moduleIOs[i].setTurnVoltage(0.0);
            } else {
              moduleIOs[i].setTurnVoltage(
                  turnFB[i].calculate(
                          turnPositions[i].getRadians(),
                          setpointStatesOptimized[i].angle.getRadians())
                      + turnFF.calculate(turnFB[i].getSetpoint().velocity));
            }

            // Update velocity based on turn error
            setpointStatesOptimized[i].speedMetersPerSecond *=
                Math.cos(turnFB[i].getPositionError());

            // Run drive controller
            double velocityRadPerSec =
                setpointStatesOptimized[i].speedMetersPerSecond
                    / (RobotConstants.get().moduleWheelDiameter() / 2);
            moduleIOs[i].setDriveVoltage(driveFF.calculate(velocityRadPerSec));

            // Log individual setpoints
            Logger.getInstance()
                .recordOutput("SwerveDriveSetpoints/" + Integer.toString(i), velocityRadPerSec);
            Logger.getInstance()
                .recordOutput(
                    "SwerveTurnSetpoints/" + Integer.toString(i),
                    setpointStatesOptimized[i].angle.getRadians());
          }

          // Log all module setpoints
          Logger.getInstance().recordOutput("SwerveModuleStates/Setpoints", setpointStates);
          Logger.getInstance()
              .recordOutput("SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
          break;

        case CHARACTERIZATION:
          for (int i = 0; i < 4; i++) {
            moduleIOs[i].setTurnVoltage(turnFB[i].calculate(turnPositions[i].getRadians(), 0.0));
            moduleIOs[i].setDriveVoltage(characterizationVoltage);
          }
          break;
      }
    }

    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      measuredStates[i] =
          new SwerveModuleState(
              moduleIOInputs[i].driveVelocity * (RobotConstants.get().moduleWheelDiameter() / 2),
              turnPositions[i]);
    }

    // Update odometry
    SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] =
          new SwerveModulePosition(
              moduleIOInputs[i].drivePosition * (RobotConstants.get().moduleWheelDiameter() / 2),
              turnPositions[i]);
    }
    if (gyroIOInputs.connected) {
      odometry.update(Rotation2d.fromDegrees(gyroIOInputs.angle), measuredPositions);
    } else {
      angle +=
          RobotConstants.get().kinematics().toChassisSpeeds(measuredStates).omegaRadiansPerSecond
              * 0.02;
      //      angle += setpoint.omegaRadiansPerSecond;
      odometry.update(new Rotation2d(angle), measuredPositions);
    }
    // Log measured states
    Logger.getInstance().recordOutput("SwerveModuleStates/Measured", measuredStates);

    // Log odometry pose
    Logger.getInstance().recordOutput("Odometry", getPose());

    // Enable/disable brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(true);
          moduleIOs[i].setDriveBrakeMode(true);
        }
      }
    } else {
      boolean stillMoving = false;
      for (int i = 0; i < 4; i++) {
        if (Math.abs(
                moduleIOInputs[i].driveVelocity * (RobotConstants.get().moduleWheelDiameter() / 2))
            > RobotConstants.get().driveMaxCoastVelocity()) {
          stillMoving = true;
        }
      }

      if (brakeMode && !stillMoving) {
        brakeMode = false;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(true);
          moduleIOs[i].setDriveBrakeMode(true);
        }
      }
    }
  }

  /**
   * Set the setpoint of the velocity controller to the given chassis speeds.
   *
   * @param speeds The desired speeds of the robot.
   */
  public void runVelocity(ChassisSpeeds speeds) {
    setpoint = speeds;
  }

  /** Stops the robot. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Get the current pose of the robot.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      modulePositions[i] =
          new SwerveModulePosition(
              moduleIOInputs[i].drivePosition * (RobotConstants.get().moduleWheelDiameter() / 2),
              new Rotation2d(moduleIOInputs[i].turnPositionAbsolute));
    }
    if (gyroIOInputs.connected) {
      odometry.resetPosition(Rotation2d.fromDegrees(gyroIOInputs.angle), modulePositions, pose);
    } else {
      odometry.resetPosition(new Rotation2d(angle), modulePositions, pose);
    }
  }

  /**
   * Drive the robot based on given velocities on the x and y-axis.
   *
   * @param x Velocity of the robot in the x direction.
   * @param y Velocity of the robot in the y direction.
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the given speeds are field relative.
   */
  public void chassisDrive(double x, double y, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, getPose().getRotation()));
    } else {
      runVelocity(new ChassisSpeeds(x, y, rot));
    }
  }

  public void runCharacterizationVolts(double volts) {
    driveMode = DriveMode.CHARACTERIZATION;
    characterizationVoltage = volts;
  }

  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (int i = 0; i < 4; i++) {
      driveVelocityAverage += moduleIOInputs[i].driveVelocity;
    }
    return driveVelocityAverage / 4.0;
  }

  private enum DriveMode {
    NORMAL,
    CHARACTERIZATION
  }
}
