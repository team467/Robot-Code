package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.characterization.SysIdFactory;
import frc.lib.io.gyro3d.GyroIO;
import frc.lib.io.gyro3d.GyroIOInputsAutoLogged;
import frc.lib.utils.LocalADStarAK;
import frc.lib.utils.RobotOdometry;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private RobotState state = RobotState.getInstance();
  private static final double inRangeDistancetoSpeaker = 3;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final RobotOdometry odometry;
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final SysIdFactory sysIdFactory;

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

    odometry = RobotOdometry.getInstance();

    sysIdFactory =
        new SysIdFactory(
            this,
            voltage -> {
              for (Module module : modules) {
                module.runCharacterization(voltage.in(Volts));
              }
            });

    Pathfinding.setPathfinder(new LocalADStarAK());
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            DriveConstants.MAX_LINEAR_SPEED,
            getModuleTranslations()[0].getNorm(),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) ->
            Logger.recordOutput(
                "PP/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("PP/TrajectorySetpoint", targetPose);
        });
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
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      lastGyroRotation = gyroInputs.yaw;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(wheelDeltas);
      lastGyroRotation = lastGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    odometry.getPoseEstimator().update(lastGyroRotation, getModulePositions());

    updateRobotState();
  }

  private void updateRobotState() {
    double distance =
        getPose()
            .getTranslation()
            .getDistance(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    Logger.recordOutput("Drive/SpeakerDistance", distance);
    state.inRange = distance < inRangeDistancetoSpeaker;
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
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

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
   * Stops the drive and turns the modules to an X arrfement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
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

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseEstimator().getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return odometry.getPoseEstimator().getEstimatedPosition().getRotation();
  }

  public double getPitchVelocity() {
    return gyroInputs.pitchRate;
  }

  public double getRollVelocity() {
    return gyroInputs.rollRate;
  }

  public Rotation3d getRotation3d() {
    return gyroInputs.rotation3d;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.getPoseEstimator().resetPosition(lastGyroRotation, getModulePositions(), pose);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }

  /**
   * Returns a command to run a quasistatic sysid test in the specified direction.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the test.
   * @see SysIdRoutine#quasistatic(SysIdRoutine.Direction)
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdFactory.quasistatic(direction);
  }

  /**
   * Returns a command to run a dynamic sysid test in the specified direction.
   *
   * @param direction The direction in which to run the test.
   * @return A command to run the test.
   * @see SysIdRoutine#dynamic(SysIdRoutine.Direction)
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdFactory.dynamic(direction);
  }
}
