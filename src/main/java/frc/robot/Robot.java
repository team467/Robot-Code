// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.utils.MakeTunable;
import frc.lib.utils.TunableWatcher;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.opmodes.AutoOpMode;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.extend.IntakeExtendSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.magicCarpet.MagicCarpetSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.shooter.ShooterSpark;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOTalonSpark;
import frc.robot.subsystems.vision.Vision;
import io.github.classgraph.ClassGraph;
import io.github.classgraph.ScanResult;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends OpModeRobot {

  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Alert canErrorAlert =
      new Alert("Init Alerts", "CAN errors detected, robot may not be controllable.", Level.HIGH);
  private final Alert lowBatteryAlert =
      new Alert("Init Alerts",
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          Level.MEDIUM);
  private final Orchestrator orchestrator;
  // Controller
  private final CommandGamepad driverController = new CommandGamepad(0);
  // Subsystems
  private Intake intake = null;
  private Drive drive;
  private Vision vision;
  private Leds leds;
  private MagicCarpet magicCarpet;
  private Indexer indexer;
  private Shooter shooter = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    setupTunables();

    if (Constants.getMode() == Constants.Mode.REAL) {
      ProcessBuilder builder = new ProcessBuilder();
      builder.command("sudo", "mount", "/dev/sda1", "/media/sda1");
      try {
        builder.start();
      } catch (IOException e) {
        DriverStationErrors.reportWarning("USB has not been mounted!", e.getStackTrace());
      }
    }

    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
      default -> Logger.recordMetadata("GitDirty", "Unknown");
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      // Running on a real robot, log to a USB stick if possible
      case REAL -> {
        Logger.addDataReceiver(new NT4Publisher());
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          Logger.addDataReceiver(new WPILOGWriter(folder));
        }
      }

      // Running a physics simulator, log to NT
      case SIM -> {
        Logger.addDataReceiver(new NT4Publisher());
      }

      // Replaying a log, set up replay source
      case REPLAY -> {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    // Start AdvantageKit logger
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();

    // Instantiate active subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2026_COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonSpark(0),
                  new ModuleIOTalonSpark(1),
                  new ModuleIOTalonSpark(2),
                  new ModuleIOTalonSpark(3));
          vision =
              new Vision(drive::addVisionMeasurement);
          leds = new Leds();
          shooter = new ShooterSpark(Optional.empty());
          magicCarpet = new MagicCarpetSpark();
          indexer = new IndexerSpark();
          intake = new Intake(new IntakeExtendSpark());

          var scheduler = Scheduler.getDefault();
          scheduler.schedule(shooter.stateMachine());
          scheduler.schedule(magicCarpet.stateMachine());
          scheduler.schedule(indexer.stateMachine());
          scheduler.schedule(intake.stateMachine());
        }

        case ROBOT_SIMBOT -> {
          shooter = new ShooterSpark(Optional.of(new ShooterSim()));
          drive =
              new Drive(
                  new GyroIO() {
                  },
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          vision = new Vision(drive::addVisionMeasurement);
          leds = new Leds();
          magicCarpet = new MagicCarpetSpark();
        }

        case ROBOT_BRIEFCASE -> {
        }
      }
    }

    orchestrator =
        new Orchestrator(
            drive, shooter, indexer);
    Autos autos = new Autos(this);
//    AutoBuilder.configure(
//        drive::getPose,
//        drive::setPose,
//        drive::getChassisVelocities,
//        drive::runVelocity,
//        new PPHolonomicDriveController(new PIDConstants(14.0, 0, 0), new PIDConstants(7, 0, 0)),
//        ppConfig,
//        () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED
//    );
//    Pathfinding.setPathfinder(new LocalADStarAK());
//    PathPlannerLogging.setLogActivePathCallback(
//        (activePath) -> {
//          Logger.recordOutput(
//              "Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
//        });
//    PathPlannerLogging.setLogTargetPoseCallback(
//        (targetPose) -> {
//          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
//        });

    // Set up auto routines
    addOpMode(RobotMode.AUTONOMOUS, "A Side", "Autos",
        () -> new AutoOpMode(autos.ppA2CycleRightRegression()));
    addOpMode(RobotMode.AUTONOMOUS, "B Side", "Autos",
        () -> new AutoOpMode(autos.ppB2CycleRightRegression()));
    addOpMode(RobotMode.AUTONOMOUS, "8 Ball", "Autos",
        () -> new AutoOpMode(autos.eightBalls()));
    publishOpModes();

    // Configure the button bindings
    configureButtonBindings();

    drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO));
  }

  private void setupTunables() {
    TunableWatcher updater = new TunableWatcher();
    try (ScanResult scanResult = new ClassGraph().enableClasspath().acceptPackages("frc.robot")
        .enableClassInfo()
        .enableFieldInfo()
        .enableAnnotationInfo().scan()) {
      for (var classInfo : scanResult.getClassesWithFieldAnnotation(MakeTunable.class.getName())) {
        try {
          for (var fieldInfo : classInfo.getFieldInfoWithAnnotation(MakeTunable.class.getName())) {
            String displayName = "";
            Unit unit = null;
            Class<?> fieldOwner = Class.forName(classInfo.getName());
            Field field = fieldOwner.getDeclaredField(fieldInfo.getName());
            field.setAccessible(true);
            Object defaultValue = field.get(null);
            for (var parameter : fieldInfo.getAllAnnotationInfo(MakeTunable.class.getName())
                .getParameterValues()) {
              if (parameter.getName().equals("name")) {
                if (parameter.getValue().equals("UseQualifiedName")) {
                  displayName = classInfo.getName() + "/" + fieldInfo.getName();
                } else {
                  displayName = (String) parameter.getValue();
                }
              } else if (parameter.getName().equals("unit")) {
                String unitName = (String) parameter.getValue();
                if (!unitName.equals("none")) {
                  unit = (Unit) Units.class.getField(unitName).get(null);
                }
              }
            }
            TunableWatcher.addTunable(fieldOwner, fieldInfo.getName(), displayName, field.getType(),
                Optional.ofNullable(unit), defaultValue);
          }
        } catch (NoSuchFieldException | IllegalAccessException | ClassNotFoundException e) {
          throw new RuntimeException(e);
        }
      }
    }
    Thread thread = new Thread(updater);
    thread.setDaemon(true);
    thread.start();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

//    new Trigger(() -> driverController.getHID().getPOV() != -1)
//        .whileTrue(drive.run(coro -> {
//          int pov = driverController.getHID().getPOV();
//
//          switch (pov) {
//            case 0 -> drive.runVelocity(new ChassisVelocities(SLOW_SPEED, 0, 0));
//            case 90 -> drive.runVelocity(new ChassisVelocities(0, -SLOW_SPEED, 0));
//            case 180 -> drive.runVelocity(new ChassisVelocities(-SLOW_SPEED, 0, 0));
//            case 270 -> drive.runVelocity(new ChassisVelocities(0, SLOW_SPEED, 0));
//            default -> drive.stop();
//          }
//        }).named("DriveWithDpad"));
  }

  public void updateContainerPeriodic() {
    RobotState.getInstance().updateLEDState();
    orchestrator.orchestratorPeriodic();
  }

  public void stopOuttake() {
    Bindings.outtake = false;
  }

  public void outtake() {
    Bindings.outtake = true;
    Bindings.intake = false;
    Bindings.intakeRetract = false;
    Bindings.shoot = false;
  }

  public void intake() {
    Bindings.outtake = false;
    Bindings.intake = true;
    Bindings.intakeRetract = false;
  }

  public void stopIntaking() {
    Bindings.intake = false;
  }

  public void shoot() {
    Bindings.outtake = false;
    Bindings.intake = false;
    Bindings.intakeRetract = false;
    Scheduler.getDefault().schedule(orchestrator.zoneBasedAim());
    orchestrator.zoneBasedShooter();
  }

  public void stopShooting() {
    Bindings.shoot = false;
  }

  public void shootManual() {
    Bindings.outtake = false;
    Bindings.intake = false;
    Bindings.intakeRetract = false;
    orchestrator.spinUpShooterHub();
  }

  public void retractIntake() {
    Bindings.outtake = false;
    Bindings.intake = false;
    Bindings.intakeRetract = true;
  }

  public void stopRetractingIntake() {
    Bindings.intakeRetract = false;
  }

  public void driveToHub() {
    Scheduler.getDefault().schedule(orchestrator.driveToHub());
  }

  /**
   * This function is called periodically during all modes.
   */
  @Override
  public void robotPeriodic() {
    long periodicBeforeStart = RobotController.getMonotonicTime();
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    long userCodeStart = RobotController.getMonotonicTime();

    Scheduler.getDefault().run();

    updateContainerPeriodic();

    drive.applyOutputs();
    leds.periodic();

    // Check CAN status
    var canStatus = RobotController.getCANStatus(CANPort.CAN_D0);
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    long userCodeEnd = RobotController.getMonotonicTime();
    Logger.AdvancedHooks.invokePeriodicAfterUser(
        userCodeEnd - userCodeStart,
        userCodeStart - periodicBeforeStart);
  }

  public void setPose() {
    drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO));
  }

  public Command followPath(String name) {
    return drive.followPath(name);
  }

  public void initPose(Pose2d pose) {
    drive.setPose(pose);
  }
}
