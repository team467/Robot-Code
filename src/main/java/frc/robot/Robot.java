// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.SLOW_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.ppConfig;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.camera2Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.lib.utils.LocalADStarAK;
import frc.robot.RobotState.IntakePosition;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.extend.IntakeExtendSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.magicCarpet.MagicCarpetSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import java.io.IOException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private static final int LOW_VOLTAGE = 9;
  private static final double loopOverrunWarningTimeout = 0.2;
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;
  private static final double lowBatteryMinCycleCount = 10;
  private static int lowBatteryCycleCount = 0;
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final Alert canErrorAlert =
      new Alert("Init Alerts", "CAN errors detected, robot may not be controllable.", Level.HIGH);
  private final Alert lowBatteryAlert =
      new Alert("Init Alerts",
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          Level.MEDIUM);
  private final RobotState state = RobotState.getInstance();
  private final LinearFilter batteryFilter = LinearFilter.movingAverage(15);
  private final Orchestrator orchestrator;
  // Controller
  private final CommandGamepad driverController = new CommandGamepad(0);
  private final CommandGamepad operatorController = new CommandGamepad(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final RobotState robotState = RobotState.getInstance();
  private final boolean isRobotOriented = true; // Workaround, change if needed
  private Intake intake = null;
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Leds leds;
  private MagicCarpet magicCarpet;
  private Indexer indexer;
  private Shooter shooter = null;
  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
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
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    // Start AdvantageKit logger
    Logger.start();

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    disabledTimer.restart();

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
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1),
                  new VisionIOPhotonVision(camera2Name, robotToCamera2));
          leds = new Leds();
          shooter = new ShooterSpark();
          magicCarpet = new MagicCarpetSpark();
          indexer = new IndexerSpark();
          intake = new Intake(new IntakeExtendSpark());
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {
                  },
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());

          leds = new Leds();
        }

        case ROBOT_BRIEFCASE -> {
        }
      }
    }

    orchestrator =
        new Orchestrator(
            drive, magicCarpet, shooter, indexer, intake, driverController);
    Autos autos = new Autos(drive, orchestrator, intake, shooter);
    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisVelocities,
        drive::runVelocity,
        new PPHolonomicDriveController(new PIDConstants(14.0, 0, 0), new PIDConstants(7, 0, 0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "RunIntakeOut", intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS));
    autoChooser.addOption("Manual A-CC", autos.ACCManualAuto());
    autoChooser.addOption("Manual A-CC-Improved", autos.ACCManuelAutoAlt());
    autoChooser.addOption("Manual A-CC-Over-Bump", autos.ACCManualAutoOverBump());
    autoChooser.addOption("Manual B-CC", autos.BCCManualAuto());
    autoChooser.addOption("Manual B-CC-Improved", autos.BCCManuelAutoAlt());
    autoChooser.addOption("Manual B-CC-Over-Bump", autos.BCCManualAutoOverBump());
    autoChooser.addOption("test path 2", drive.getAutonomousCommand("test path 2"));
    autoChooser.addOption("8 Ball Auto", autos.EightBalls());

    // Drive SysId
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // pp

    autoChooser.addOption("A-side", autos.ppA2CycleRightRegression());
    autoChooser.addOption("B-side", autos.ppB2CycleRightRegression());
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .start()
        .onTrue(
            Command.noRequirements(
                    _ ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO)))
                .named("Set Initial Drive Pose"));

    new Trigger(() -> driverController.getHID().getPOV() != -1)
        .whileTrue(drive.run(coro -> {
          int pov = driverController.getHID().getPOV();

          switch (pov) {
            case 0 -> drive.runVelocity(new ChassisVelocities(SLOW_SPEED, 0, 0));
            case 90 -> drive.runVelocity(new ChassisVelocities(0, -SLOW_SPEED, 0));
            case 180 -> drive.runVelocity(new ChassisVelocities(-SLOW_SPEED, 0, 0));
            case 270 -> drive.runVelocity(new ChassisVelocities(0, SLOW_SPEED, 0));
            default -> drive.stop();
          }
        }).named("DriveWithDpad"));
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
    orchestrator.zoneBasedAim();
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
    orchestrator.driveToHub();
  }

  /**
   * This function is called periodically during all modes.
   */
  @Override
  public void robotPeriodic() {
    updateContainerPeriodic();

    // Check CAN status
    var canStatus = RobotController.getCANStatus(CANPort.CAN_D0);
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Low battery alert
    lowBatteryCycleCount += 1;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
    }

    // If robot has low battery, lowbatteryalert will be set to true
    state.lowBatteryAlert =
        batteryFilter.calculate(RobotController.getBatteryVoltage()) < LOW_VOLTAGE;
  }
}
