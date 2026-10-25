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
import static org.wpilib.units.Units.Amps;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.lib.mechanisms.flywheels.ControlledFlywheel;
import frc.lib.mechanisms.flywheels.Flywheel;
import frc.lib.mechanisms.flywheels.ReversibleFlywheel;
import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.lib.utils.LocalADStarAK;
import frc.robot.RobotState.IntakePosition;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonSpark;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import frc.robot.subsystems.intake.extend.IntakeExtendIOSparkMax;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOKraken;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.magicCarpet.MagicCarpetConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.CustomTriggers;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.XboxController;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Orchestrator orchestrator;
  // Controller
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();
  private final boolean isRobotOriented = true; // Workaround, change if needed
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Leds leds;
  private Flywheel magicCarpet;
  private ReversibleFlywheel indexer;
  private ControlledFlywheel shooter;
  private IntakeRollers intakeRollers;
  private IntakeExtend intakeExtend;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
          shooter = new Shooter();
          magicCarpet = new SparkFlywheel("Magic Carpet", CANPort.CAN_D0,
              Schematic.magicCarpetCanId,
              MagicCarpetConstants.MOTOR_INVERTED, MagicCarpetConstants.BELT_VOLTS,
              MagicCarpetConstants.CURRENT_LIMIT);
          indexer = new SparkFlywheel("Indexer", CANPort.CAN_D0,
              Schematic.indexerFeedupCanId, false, IndexerConstants.FEEDUP_VOLT,
              Amps.of(30));
          intakeRollers = new IntakeRollers(new IntakeRollersIOKraken());
          intakeExtend =
              new IntakeExtend(new IntakeExtendIOSparkMax(), operatorController.rightTrigger());
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
          //          leds = new Leds();
          //    hopperBelt = new HopperBelt(new HopperBeltSparkMax());
          leds = new Leds();
          shooter = new Shooter(new ShooterIOSparkMax(false));
        }
      }
    }

    // Instantiate missing subsystems
    intake = new Intake(intakeRollers, intakeExtend);

    orchestrator =
        new Orchestrator(
            drive, magicCarpet, shooter, indexer, intake, intakeRollers, driverController);
    Autos autos = new Autos(drive, orchestrator, intake, intakeRollers, shooter);
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

    autoChooser.addOption(
        "runCharacterizationQuasistatic", shooter.sysIdQuasistatic(Direction.kForward));
    autoChooser.addOption("runCharacterizationDynamic", shooter.sysIdDynamic(Direction.kForward));

    autoChooser.addOption(
        "runCharacterizationQuasistaticReverse", shooter.sysIdQuasistatic(Direction.kReverse));
    autoChooser.addOption(
        "runCharacterizationDynamicReverse", shooter.sysIdDynamic(Direction.kReverse));

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

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link org.wpilib.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link org.wpilib.command2.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    indexer.setDefaultCommand(indexer.stop());
    shooter.setDefaultCommand(shooter.stop());

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0 degrees when A button is held
    Trigger intaking = new Trigger(() -> RobotState.getInstance().intaking);
    intaking.onTrue(
        Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 0.2)));
    intaking.onFalse(
        Commands.runOnce(() -> driverController.setRumble(RumbleType.kBothRumble, 0.0)));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
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
    driverController.x().toggleOnTrue(orchestrator.zoneBasedAim());
    driverController.y().toggleOnTrue(intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS));
    driverController
        .leftBumper()
        .and(operatorController.pov(180))
        .whileTrue(intakeExtend.runIntakeExtendVolts(-4))
        .onFalse(intakeExtend.stopExtendingCommand());
    CustomTriggers.toggleIntakeUp(
            driverController.leftBumper(),
            () -> RobotState.getInstance().intakePosition == IntakePosition.DEPLOYED)
        .and(() -> !operatorController.pov(180).getAsBoolean())
        .toggleOnTrue(intakeExtend.extendToAngle(IntakeConstants.COLLAPSE_POS));
    CustomTriggers.toggleIntakeDown(
            driverController.leftBumper(),
            () -> RobotState.getInstance().intakePosition == IntakePosition.STOWED)
        .and(() -> !operatorController.pov(180).getAsBoolean())
        .toggleOnTrue(intake.extendToAngle(IntakeConstants.EXTEND_POS));

    //         VERY IMPORTANT BECAUSE COMMAND GROUP DOESN'T MESH WITH SHOOTING DON'T COMBINE
    // driverController.leftTrigger(0.2).toggleOnTrue(intake.runIntakeMotor());

    driverController.rightTrigger(0.1).toggleOnTrue(orchestrator.feedUp());
    driverController.leftTrigger(0.2).toggleOnTrue(intake.runIntakeMotor());

    driverController
        .a()
        .and(operatorController.pov(180))
        .onTrue(intakeExtend.resetExtendPosition());
    driverController
        .rightBumper()
        .and(() -> !operatorController.pov(180).getAsBoolean())
        .toggleOnTrue(orchestrator.driveToHub());
    driverController
        .rightBumper()
        .and(operatorController.pov(180))
        .whileTrue(intakeExtend.runIntakeExtendVolts(4))
        .onFalse(intakeExtend.stopExtendingCommand());
    operatorController.leftTrigger(0.1).toggleOnTrue(orchestrator.spinUpShooterTest());
    operatorController
        .rightTrigger(0.1)
        .and(() -> !operatorController.pov(0).getAsBoolean())
        .toggleOnTrue(orchestrator.zoneBasedShooter());
    operatorController
        .rightTrigger(0.1)
        .and(operatorController.pov(0))
        .toggleOnTrue(orchestrator.spinUpShooterHub());
    operatorController.y().whileTrue(indexer.reverse());
    operatorController.x().whileTrue(intakeRollers.outtake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void robotPeriodic() {
    RobotState.getInstance().updateLEDState();
    orchestrator.orchestratorPeriodic();
  }

  private Command rumblePulse(double intensity, double seconds) {
    return Commands.sequence(
        Commands.runOnce(
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, intensity)),
        Commands.waitSeconds(seconds),
        Commands.runOnce(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }
}
