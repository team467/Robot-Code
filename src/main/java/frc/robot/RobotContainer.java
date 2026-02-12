// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOPhysical;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.hopperbelt.HopperBeltIO;
import frc.robot.subsystems.hopperbelt.HopperBeltSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Subsystem subsystem;
  private Drive drive;
  private Vision vision;
  private Leds leds;
  private HopperBelt hopperBelt;
  private Intake intake;
  private Indexer indexer;
  private final Orchestrator orchestrator;
  private Shooter shooter;
  private Climber climber;
  private RobotState robotState = RobotState.getInstance();
  private boolean isRobotOriented = true; // Workaround, change if needed

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoChooser choreoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate active subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2025_COMP -> {
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
                  new VisionIOPhotonVision(camera2Name, robotToCamera2),
                  new VisionIOPhotonVision(camera3Name, robotToCamera3));
          leds = new Leds();
        }
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
                  new VisionIOPhotonVision(camera2Name, robotToCamera2),
                  new VisionIOPhotonVision(camera3Name, robotToCamera3));
          leds = new Leds();
          shooter = new Shooter(new ShooterIOSparkMax());
          hopperBelt = new HopperBelt(new HopperBeltSparkMax());
          indexer = new Indexer(new IndexerIOSparkMax());
          climber = new Climber(new ClimberIOPhysical());
          intake = new Intake(new IntakeIOSparkMax(), operatorController.rightTrigger());
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());

          leds = new Leds();
        }

        case ROBOT_BRIEFCASE -> {
          leds = new Leds();
          intake =
              new Intake(
                  new IntakeIOSparkMax(),
                  new BooleanSupplier() {
                    @Override
                    public boolean getAsBoolean() {
                      return true;
                    }
                  });
          //    hopperBelt = new HopperBelt(new HopperBeltSparkMax());
          //    shooter = new Shooter(new ShooterIOSparkMax());
        }
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {}, () -> false);
    }
    if (hopperBelt == null) {
      hopperBelt = new HopperBelt(new HopperBeltIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }

    orchestrator =
        new Orchestrator(
            drive,
            hopperBelt,
            shooter,
            indexer,
            intake,
            driverController); // Commented Out Intake --> Add Back
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    choreoChooser = new AutoChooser();
    Autos autos = new Autos(drive);
    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("test path", autos.testPath());
    autoChooser.addOption("test path 2", drive.getAutonomousCommand("test path 2"));

    SmartDashboard.putData("Choreo Autos", choreoChooser);

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
    // pathplanner
    NamedCommands.registerCommand(
        "startShooter", Commands.parallel(orchestrator.preloadBalls(), orchestrator.prepShooter()));
    NamedCommands.registerCommand("shoot", orchestrator.shootBalls());
    NamedCommands.registerCommand("shootClimb", orchestrator.shootBallsonClimb());
    NamedCommands.registerCommand("shootDistance", orchestrator.shootBallsAtDistance());
    NamedCommands.registerCommand("extend hopper and intake", intake.extendAndIntake());
    NamedCommands.registerCommand(
        "stopIntake",
        Commands.sequence(
            intake.collapseAndIntake(), Commands.waitSeconds(0.3), intake.stopIntakeCommand()));
    NamedCommands.registerCommand("climb", Commands.none());
    autoChooser.addOption("CA-1C-O-Climb", autos.CenterA());
    autoChooser.addOption("CC-1C-D-Climb", autos.CenterC());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held

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
        .whileTrue(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    if (Constants.getRobot() == Constants.RobotType.ROBOT_2026_COMP) {
      driverController.rightBumper().whileTrue(orchestrator.shootBalls());
    }

    //    driverController.x().onTrue(shooter.setTargetVelocity(250)).onFalse(shooter.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public AutoChooser getAutoChooser() {
    return choreoChooser;
  }

  public void robotPeriodic() {
    RobotState.getInstance().updateLEDState();
    orchestrator.OrchestratorPeriodic();
  }
}
