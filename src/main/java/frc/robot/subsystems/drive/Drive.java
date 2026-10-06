package frc.robot.subsystems.drive;

import static org.wpilib.framework.RobotBase.isDisabled;
import static org.wpilib.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.RobotState;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.wpiutil.Alert; // 2027 migration: Alert moved from HAL/wpilib into wpiutil
import org.wpilib.wpiutil.Alert.Level;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleState;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
// 2027 migration: SysIdRoutine below is still the Commands v2 class (no v3 equivalent
// shipped yet as of alpha-7). Its Mechanism.Mechanism(...) constructor's third argument
// expects a v2 Subsystem, but this class now implements the v3 Mechanism interface
// (see class declaration) instead. Passing `this` there is a real type mismatch that
// needs a team decision, not a mechanical fix — see summary below.
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive implements Mechanism {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  // private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Choreo PIDs
  private final PIDController xController = new PIDController(3.0, 0.0, 0.5);
  private final PIDController yController = new PIDController(3.0, 0.0, 0.5);
  private final PIDController headingController = new PIDController(2.5, 0.0, 0);

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

    // Start odometry thread
    OdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner

    // Configure SysId
//    sysId =
//        new SysIdRoutine(
//            new SysIdRoutine.Config(
//                null,
//                null,
//                null,
//                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
//            // 2027 migration: `this` no longer satisfies this constructor's expected type
//            // (v2 Subsystem) now that Drive implements the v3 Mechanism interface. Needs a
//            // team decision — see summary.
//            new SysIdRoutine.Mechanism(
//                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }


  public void periodic() {
    logCameraPositions(); // uncomment to show camera positions in advantage scope
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();
    // Stop moving when disabled
    // 2027 migration: DriverStation was split into MatchState/RobotState (alpha 5).
    // isDisabled() reads robot state, so this likely needs to move to RobotState.isDisabled()
    // — verify the exact package/class against your target alpha before changing this.
    if (isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance
                    - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    //gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    var desaturatedStates = SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(desaturatedStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

//  public Command getAutonomousCommand(String Path) {
//    try {
//      // Load the path you want to follow using its name in the GUI
//      PathPlannerPath path = PathPlannerPath.fromPathFile(Path);
//
//      // Create a path following command using AutoBuilder. This will also trigger event markers.
//      // 2027 migration: PathPlanner is a vendor library — it needs its own 2027-compatible
//      // vendordep release. Confirm the version you're on returns a command type compatible
//      // with org.wpilib.command3.Command (this method's return type); pre-2027 PathPlanner
//      // versions return a Commands v2 command, which will not satisfy this signature.
//      return AutoBuilder.followPath(path);
//    } catch (Exception e) {
//      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
//      // 2027 migration: Commands.none() was already unresolved in the original file (no
//      // Commands import) — pre-existing bug, not introduced by this migration. If you want a
//      // no-op Command here, confirm what Commands v3's equivalent factory method is called
//      // and import it explicitly; v3's API doesn't mirror v2's Commands class 1:1.
//      return Commands.none();
//    }
//  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle().get();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

//  /** Returns a command to run a quasistatic test in the specified direction. */
//  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//    return run(() -> runCharacterization(0.0))
//        .withTimeout(1.0)
//        .andThen(sysId.quasistatic(direction));
//  }
//
//  /** Returns a command to run a dynamic test in the specified direction. */
//  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
//  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveVelocities/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  private void logCameraPositions() {
    Logger.recordOutput(
        "CameraPos/camera0position",
        new Pose3d(getPose().getX(), getPose().getY(), 0.0, new Rotation3d(getRotation()))
            .transformBy(VisionConstants.robotToCamera0));
    Logger.recordOutput(
        "CameraPos/camera1position",
        new Pose3d(getPose().getX(), getPose().getY(), 0.0, new Rotation3d(getRotation()))
            .transformBy(VisionConstants.robotToCamera1));
    Logger.recordOutput(
        "CameraPos/camera2position",
        new Pose3d(getPose().getX(), getPose().getY(), 0.0, new Rotation3d(getRotation()))
            .transformBy(VisionConstants.robotToCamera2));
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}