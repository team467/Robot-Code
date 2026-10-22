package first.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.pathplanner.lib.command3.AutoBuilder;
import com.pathplanner.lib.command3.NamedCommands;
import first.robot.constants.FieldConstants;
import first.robot.mechanisms.Intake.Extend.IntakeExtendIO;
import first.robot.mechanisms.Intake.Extend.IntakeExtendIOSim;
import first.robot.mechanisms.Intake.Extend.IntakeExtendIOSparkMax;
import first.robot.mechanisms.Intake.Intake;
import first.robot.mechanisms.Intake.Rollers.IntakeRollerIO;
import first.robot.mechanisms.Intake.Rollers.IntakeRollerIOSim;
import first.robot.mechanisms.Intake.Rollers.IntakeRollerIOTalonFX;
import first.robot.mechanisms.indexer.Indexer;
import first.robot.mechanisms.indexer.IndexerIO;
import first.robot.mechanisms.indexer.IndexerIOSim;
import first.robot.mechanisms.indexer.IndexerIOSparkMax;
import first.robot.mechanisms.magicCarpet.MagicCarpet;
import first.robot.mechanisms.magicCarpet.MagicCarpetIO;
import first.robot.mechanisms.magicCarpet.MagicCarpetIOSim;
import first.robot.mechanisms.magicCarpet.MagicCarpetIOSparkMax;
import first.robot.mechanisms.shooter.Shooter;
import first.robot.mechanisms.shooter.ShooterConstants;
import first.robot.mechanisms.shooter.ShooterIO;
import first.robot.mechanisms.shooter.ShooterIOSim;
import first.robot.mechanisms.shooter.ShooterIOSparkMax;
import first.robot.mechanisms.swerve.Drive;
import first.robot.mechanisms.swerve.GyroIO;
import first.robot.mechanisms.swerve.GyroIOPigeon2;
import first.robot.mechanisms.swerve.ModuleIO;
import first.robot.mechanisms.swerve.ModuleIOSim;
import first.robot.mechanisms.stereo.StereoVision;
import first.robot.mechanisms.stereo.StereoVisionIO;
import first.robot.mechanisms.stereo.StereoVisionIOSim;
import first.robot.mechanisms.swerve.ModuleIOTalonSpark;
import first.robot.mechanisms.vision.Vision;
import first.robot.mechanisms.vision.VisionConstants;
import first.robot.mechanisms.vision.VisionIO;
import first.robot.mechanisms.vision.VisionIOPhotonVision;
import first.robot.mechanisms.vision.VisionIOPhotonVisionSim;
import first.robot.commands.auto.Autos;
import first.robot.constants.IntakeConstants;
import first.robot.opmodes.auto.CommandOpMode;
import first.robot.opmodes.auto.PathPlannerAutoOpMode;
import java.io.File;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.NotLogged;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.RobotController;

@Logged
public class Robot extends OpModeRobot {
  @NotLogged public final Drive drive;
  @NotLogged public final Intake intake;
  @NotLogged public final Vision vision;
  @NotLogged public final StereoVision stereoVision;
  @NotLogged public final Shooter shooter;
  @NotLogged public final MagicCarpet magicCarpet;
  @NotLogged public final Indexer indexer;
  @NotLogged public final CommandGamepad driverController = new CommandGamepad(0);
  @NotLogged public final CommandGamepad operatorController = new CommandGamepad(1);
  @NotLogged public final Orchestrator orchestrator;

  private final Scheduler scheduler = Scheduler.getDefault();

  public Robot() {
    startLogger();

    switch (Constants.currentMode) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonSpark(0),
                new ModuleIOTalonSpark(1),
                new ModuleIOTalonSpark(2),
                new ModuleIOTalonSpark(3));
        intake = new Intake(new IntakeExtendIOSparkMax(), new IntakeRollerIOTalonFX());
        shooter = new Shooter(new ShooterIOSparkMax());
        magicCarpet = new MagicCarpet(new MagicCarpetIOSparkMax());
        indexer = new Indexer(new IndexerIOSparkMax());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2));
        stereoVision = new StereoVision(new StereoVisionIO() {}, drive::getPose);
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeExtendIOSim(), new IntakeRollerIOSim());
        shooter = new Shooter(new ShooterIOSim());
        magicCarpet = new MagicCarpet(new MagicCarpetIOSim());
        indexer = new Indexer(new IndexerIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose));
        stereoVision = new StereoVision(new StereoVisionIOSim(drive::getPose), drive::getPose);
      }
      default -> {
        drive =
            new Drive(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeExtendIO() {}, new IntakeRollerIO() {});
        shooter = new Shooter(new ShooterIO() {});
        magicCarpet = new MagicCarpet(new MagicCarpetIO() {});
        indexer = new Indexer(new IndexerIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIO() {});
        stereoVision = new StereoVision(new StereoVisionIO() {}, drive::getPose);
      }
    }

    orchestrator =
        new Orchestrator(drive, magicCarpet, shooter, indexer, intake, driverController);

    drive.setDefaultCommand(drive.idle());
    shooter.setDefaultCommand(shooter.idle());
    intake.setDefaultCommand(intake.idle());
    indexer.setDefaultCommand(indexer.idle());
    magicCarpet.setDefaultCommand(magicCarpet.runWhile(indexer::isRunning));

    registerPathPlannerAutos();
    registerCommandOpModes();

    AutoLogOutputManager.addObject(this);
  }

  private void registerPathPlannerAutos() {
    NamedCommands.registerCommand("Intake", intake.intake());
    NamedCommands.registerCommand("Stow", intake.stow());
    NamedCommands.registerCommand("HomeIntake", intake.home());
    NamedCommands.registerCommand("RunMagicCarpet", magicCarpet.run());
    NamedCommands.registerCommand("RunIndexer", indexer.run());
    NamedCommands.registerCommand("ReverseIndexer", indexer.reverse());
    NamedCommands.registerCommand(
        "ShootAtHub",
        Command.noRequirements(
                coroutine -> FieldConstants.targetHub().ifPresent(hub -> coroutine.await(shootAt(hub))))
            .named("Robot.ShootAtHub"));

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      addOpMode(
          RobotMode.AUTONOMOUS,
          autoName,
          "PathPlanner",
          () -> new PathPlannerAutoOpMode(autoName));
    }
    publishOpModes();
  }

  private void registerCommandOpModes() {
    Autos autos = new Autos(drive, orchestrator, intake, shooter);

    String manual = "Manual Autos";
    addCommandOpMode("Manual A-CC", manual, autos::ACCManualAuto);
    addCommandOpMode("Manual A-CC-Improved", manual, autos::ACCManuelAutoAlt);
    addCommandOpMode("Manual A-CC-Over-Bump", manual, autos::ACCManualAutoOverBump);
    addCommandOpMode("Manual B-CC", manual, autos::BCCManualAuto);
    addCommandOpMode("Manual B-CC-Improved", manual, autos::BCCManuelAutoAlt);
    addCommandOpMode("Manual B-CC-Over-Bump", manual, autos::BCCManualAutoOverBump);
    addCommandOpMode("8 Ball Auto", manual, autos::EightBalls);

    String pp = "PathPlanner Cycles";
    addCommandOpMode("A-side", pp, autos::ppA2CycleRightRegression);
    addCommandOpMode("B-side", pp, autos::ppB2CycleRightRegression);
    addCommandOpMode("A-side 1 Cycle", pp, autos::ppA1CycleRegression);
    addCommandOpMode("B-side 1 Cycle", pp, autos::ppB1CycleRegression);
    addCommandOpMode("Depot+Outpost", pp, autos::ppDepot);
    addCommandOpMode(
        "Depot+Outpost (from center)", pp, autos::ppDepotOutpost);

    addCommandOpMode(
        "RunIntakeOut", "Utility", () -> intake.extendToAngleAndIntake(IntakeConstants.EXTEND_POS));

    String sysId = "Characterization";
    addCommandOpMode(
        "Shooter SysId (Quasistatic Forward)",
        sysId,
        () -> shooter.sysIdQuasistatic(Shooter.SysIdDirection.FORWARD));
    addCommandOpMode(
        "Shooter SysId (Quasistatic Reverse)",
        sysId,
        () -> shooter.sysIdQuasistatic(Shooter.SysIdDirection.REVERSE));
    addCommandOpMode(
        "Shooter SysId (Dynamic Forward)",
        sysId,
        () -> shooter.sysIdDynamic(Shooter.SysIdDirection.FORWARD));
    addCommandOpMode(
        "Shooter SysId (Dynamic Reverse)",
        sysId,
        () -> shooter.sysIdDynamic(Shooter.SysIdDirection.REVERSE));
    addCommandOpMode(
        "Drive Wheel Radius Characterization", sysId, drive::wheelRadiusCharacterization);
    addCommandOpMode(
        "Drive Simple FF Characterization", sysId, drive::feedforwardCharacterization);
    publishOpModes();
  }

  private void addCommandOpMode(String name, String group, Supplier<Command> command) {
    addOpMode(RobotMode.AUTONOMOUS, name, group, () -> new CommandOpMode(command));
  }

  private static void startLogger() {
    Logger.recordMetadata("ProjectName", "2027-wpilib");

    switch (Constants.currentMode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter(realLogDirectory()));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.start();
  }

  private static String realLogDirectory() {
    File usb = new File("/u");
    if (usb.isDirectory() && usb.canWrite()) {
      return "/u/logs";
    }
    return "/home/systemcore/logs";
  }

  @Override
  public void robotPeriodic() {
    long periodicBeforeStart = RobotController.getMonotonicTime();
    Logger.AdvancedHooks.invokePeriodicBeforeUser();
    long userCodeStart = RobotController.getMonotonicTime();

    scheduler.run();

    drive.applyOutputs();
    intake.applyOutputs();
    shooter.applyOutputs();
    magicCarpet.applyOutputs();
    indexer.applyOutputs();

    Epilogue.update(this);

    long userCodeEnd = RobotController.getMonotonicTime();
    Logger.AdvancedHooks.invokePeriodicAfterUser(
        userCodeEnd - userCodeStart, userCodeStart - periodicBeforeStart);
  }

  public Command shootAt(Pose2d targetPose) {
    return Command.noRequirements(
            coroutine -> {
              coroutine.await(drive.aimAt(targetPose));

              coroutine.fork(
                  shooter.shootAtHub(
                      () ->
                          Meters.of(
                              drive
                                  .getPose()
                                  .transformBy(ShooterConstants.kShooterOffsetFromRobotCenter)
                                  .getTranslation()
                                  .getDistance(targetPose.getTranslation()))));
              coroutine.waitUntil(shooter::isReadyToShoot, Seconds.of(1.0));
              coroutine.awaitAll(intake.agitate(), indexer.run());
            })
        .named("Robot.ShootAt[" + targetPose + "]");
  }
}
