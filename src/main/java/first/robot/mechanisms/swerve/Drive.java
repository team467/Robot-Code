package first.robot.mechanisms.swerve;

import com.pathplanner.lib.command3.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

public class Drive implements Mechanism {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);

  private final PIDController headingController =
      new PIDController(DriveConstants.headingKp, 0.0, DriveConstants.headingKd);

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

    OdometryThread.getInstance().start();

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(DriveConstants.headingToleranceRad);

    configurePathPlanner();

    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (var module : modules) {
      sampleCount = Math.min(sampleCount, module.getOdometryPositions().length);
    }
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
  }

  public void applyOutputs() {
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }
  }

  @Override
  public Command idle() {
    return runRepeatedly(this::stop).withPriority(Command.LOWEST_PRIORITY).named("Drive.Idle");
  }

  public Command driveRobotRelative(Supplier<ChassisVelocities> velocities) {
    return runRepeatedly(() -> runVelocity(velocities.get())).named("Drive.RobotRelative");
  }

  public Command driveFieldRelative(Supplier<ChassisVelocities> velocities) {
    return runRepeatedly(() -> runVelocity(velocities.get().toRobotRelative(getRotation())))
        .named("Drive.FieldRelative");
  }

  public Command driverControl(CommandGamepad controller) {
    return driveFieldRelative(
        () -> {
          Translation2d linear = getLinearVelocity(controller);
          double omega = MathUtil.applyDeadband(-controller.getRightX(), DriveConstants.deadband);
          omega = Math.copySign(omega * omega, omega);
          return driverToField(
              new ChassisVelocities(
                  linear.getX(), linear.getY(), omega * getMaxAngularSpeedRadPerSec()));
        });
  }

  public Command aimAt(Pose2d targetPose) {
    return run(coroutine -> {
          headingController.reset();
          do {
            double omega = aimOmega(targetPose);
            runVelocity(new ChassisVelocities(0.0, 0.0, omega).toRobotRelative(getRotation()));
            coroutine.yield();
          } while (!headingController.atSetpoint());
          stop();
        })
        .named("Drive.AimAt");
  }

  public Command aimAssist(CommandGamepad controller, Supplier<Pose2d> targetPose) {
    return run(coroutine -> {
          headingController.reset();
          while (true) {
            Translation2d linear = getLinearVelocity(controller);
            ChassisVelocities driver =
                driverToField(new ChassisVelocities(linear.getX(), linear.getY(), 0.0));
            double omega = aimOmega(targetPose.get());
            runVelocity(
                new ChassisVelocities(driver.vx, driver.vy, omega).toRobotRelative(getRotation()));
            coroutine.yield();
          }
        })
        .named("Drive.AimAssist");
  }

  public Command followPath(String pathName) {
    return Command.noRequirements(
            coroutine -> {
              Optional<PathPlannerPath> path = loadPath(pathName);
              if (path.isPresent()) {
                coroutine.await(AutoBuilder.followPath(path.get()));
              }
            })
        .named("Drive.FollowPath[" + pathName + "]");
  }

  private static Optional<PathPlannerPath> loadPath(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStationErrors.reportError(
          "Failed to load PathPlanner path \"" + pathName + "\": " + e.getMessage(),
          e.getStackTrace());
      return Optional.empty();
    }
  }

  private void configurePathPlanner() {
    RobotConfig ppConfig =
        new RobotConfig(
            DriveConstants.robotMassKg,
            DriveConstants.robotMOI,
            new ModuleConfig(
                DriveConstants.wheelRadiusMeters,
                DriveConstants.maxSpeedMetersPerSec,
                DriveConstants.wheelCOF,
                DriveConstants.driveGearbox,
                DriveConstants.driveMotorReduction,
                DriveConstants.driveMotorCurrentLimit,
                1),
            DriveConstants.moduleTranslations);

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisVelocities,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(DriveConstants.ppTranslationKp, 0.0, DriveConstants.ppTranslationKd),
            new PIDConstants(DriveConstants.ppRotationKp, 0.0, DriveConstants.ppRotationKd)),
        ppConfig,
        Drive::isRedAlliance,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("Odometry/Trajectory", poses.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Odometry/TrajectorySetpoint", pose));
  }

  public Command joystickDriveAtAngle(CommandGamepad controller, Supplier<Rotation2d> heading) {
    return run(coroutine -> {
          headingController.reset();
          while (true) {
            Translation2d linear = getLinearVelocity(controller);
            ChassisVelocities driver =
                driverToField(new ChassisVelocities(linear.getX(), linear.getY(), 0.0));
            runVelocity(
                new ChassisVelocities(driver.vx, driver.vy, headingOmega(heading.get()))
                    .toRobotRelative(getRotation()));
            coroutine.yield();
          }
        })
        .named("Drive.JoystickDriveAtAngle");
  }

  public Command driveWithDpad(Supplier<POVDirection> pov) {
    return run(coroutine -> {
          while (true) {
            double speed = DriveConstants.dpadSpeedMetersPerSec;
            double diagonal = speed / Math.sqrt(2.0);
            ChassisVelocities velocities =
                switch (pov.get()) {
                  case UP -> new ChassisVelocities(speed, 0.0, 0.0);
                  case DOWN -> new ChassisVelocities(-speed, 0.0, 0.0);
                  case LEFT -> new ChassisVelocities(0.0, speed, 0.0);
                  case RIGHT -> new ChassisVelocities(0.0, -speed, 0.0);
                  case UP_LEFT -> new ChassisVelocities(diagonal, diagonal, 0.0);
                  case UP_RIGHT -> new ChassisVelocities(diagonal, -diagonal, 0.0);
                  case DOWN_LEFT -> new ChassisVelocities(-diagonal, diagonal, 0.0);
                  case DOWN_RIGHT -> new ChassisVelocities(-diagonal, -diagonal, 0.0);
                  case CENTER -> new ChassisVelocities();
                };
            runVelocity(velocities);
            coroutine.yield();
          }
        })
        .named("Drive.DriveWithDpad");
  }

  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    return run(coroutine -> {
          velocitySamples.clear();
          voltageSamples.clear();

          Timer timer = new Timer();
          timer.start();
          while (timer.get() < DriveConstants.ffStartDelay) {
            runCharacterization(0.0);
            coroutine.yield();
          }

          timer.reset();
          while (true) {
            double voltage = timer.get() * DriveConstants.ffRampRate;
            runCharacterization(voltage);
            velocitySamples.add(getFFCharacterizationVelocity());
            voltageSamples.add(voltage);
            coroutine.yield();
          }
        })
        .whenCanceled(
            () -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              var formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            })
        .named("Drive.FeedforwardCharacterization");
  }

  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(DriveConstants.wheelRadiusRampRate);
    double[] startPositions = new double[4];
    double[] gyroDelta = new double[1];
    return run(coroutine -> {
          limiter.reset(0.0);
          Timer timer = new Timer();
          timer.start();
          boolean measuring = false;
          Rotation2d lastAngle = getRotation();
          while (true) {
            double speed = limiter.calculate(DriveConstants.wheelRadiusMaxVelocity);
            runVelocity(new ChassisVelocities(0.0, 0.0, speed));

            if (!measuring && timer.get() >= 1.0) {
              measuring = true;
              System.arraycopy(getWheelRadiusCharacterizationPositions(), 0, startPositions, 0, 4);
              lastAngle = getRotation();
              gyroDelta[0] = 0.0;
            } else if (measuring) {
              Rotation2d rotation = getRotation();
              gyroDelta[0] += Math.abs(rotation.minus(lastAngle).getRadians());
              lastAngle = rotation;
            }
            coroutine.yield();
          }
        })
        .whenCanceled(
            () -> {
              double[] positions = getWheelRadiusCharacterizationPositions();
              double wheelDelta = 0.0;
              for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - startPositions[i]) / 4.0;
              }
              double wheelRadius = (gyroDelta[0] * DriveConstants.driveBaseRadius) / wheelDelta;

              var formatter = new DecimalFormat("#0.000");
              System.out.println("********** Wheel Radius Characterization Results **********");
              System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
              System.out.println("\tGyro Delta: " + formatter.format(gyroDelta[0]) + " radians");
              System.out.println(
                  "\tWheel Radius: "
                      + formatter.format(wheelRadius)
                      + " meters, "
                      + formatter.format(Units.metersToInches(wheelRadius))
                      + " inches");
            })
        .named("Drive.WheelRadiusCharacterization");
  }

  public Command lock() {
    return run(coroutine -> {
          stopWithX();
          coroutine.park();
        })
        .named("Drive.Lock");
  }

  public Command resetHeading() {
    return Command.noRequirements(
            coroutine -> {
              Rotation2d forward = isRedAlliance() ? Rotation2d.PI : Rotation2d.ZERO;
              setPose(new Pose2d(getPose().getTranslation(), forward));
            })
        .named("Drive.ResetHeading");
  }

  public void runVelocity(ChassisVelocities speeds) {
    ChassisVelocities discreteSpeeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discreteSpeeds),
            DriveConstants.maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    SwerveModuleVelocity[] optimizedStates = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle().orElse(Rotation2d.ZERO);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private double aimOmega(Pose2d targetPose) {
    Pose2d robot = getPose();
    return headingOmega(
        new Rotation2d(
            Math.atan2(targetPose.getY() - robot.getY(), targetPose.getX() - robot.getX())));
  }

  private double headingOmega(Rotation2d heading) {
    return Math.clamp(
        headingController.calculate(getRotation().getRadians(), heading.getRadians()),
        -getMaxAngularSpeedRadPerSec(),
        getMaxAngularSpeedRadPerSec());
  }

  private Translation2d getLinearVelocity(CommandGamepad controller) {
    double x = -controller.getLeftY();
    double y = -controller.getLeftX();
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.deadband);
    if (magnitude == 0.0) {
      return Translation2d.ZERO;
    }
    double scale = Math.min(magnitude * magnitude, 1.0) / Math.hypot(x, y);
    return new Translation2d(x * scale, y * scale).times(DriveConstants.maxSpeedMetersPerSec);
  }

  private ChassisVelocities driverToField(ChassisVelocities velocities) {
    return isRedAlliance()
        ? new ChassisVelocities(-velocities.vx, -velocities.vy, velocities.omega)
        : velocities;
  }

  private static boolean isRedAlliance() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleVelocity[] getModuleVelocities() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getVelocity();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleVelocities());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "Drive/Tilted")
  public boolean isTilted() {
    return Math.abs(gyroInputs.roll) > DriveConstants.rollThreshold
        || Math.abs(gyroInputs.pitch) > DriveConstants.pitchThreshold;
  }

  public boolean inZone(Rectangle2d zone) {
    return zone.contains(getPose().getTranslation());
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.maxSpeedMetersPerSec / DriveConstants.driveBaseRadius;
  }
}
