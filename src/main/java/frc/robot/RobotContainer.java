// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.ElevatorPosition;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.FieldAlignment;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.algae.AlgaeEffectorIO;
import frc.robot.subsystems.algae.AlgaeEffectorIOPhysical;
import frc.robot.subsystems.algae.AlgaeEffectorIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.coral.CoralEffectorIO;
import frc.robot.subsystems.coral.CoralEffectorIOSparkMAX;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOPhysical;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
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
  private AlgaeEffector algae;
  private CoralEffector coral;
  private Climber climber;
  private Elevator elevator;
  private FieldAlignment fieldAlignment;
  private Orchestrator orchestrator;
  private RobotState robotState = RobotState.getInstance();
  private boolean isRobotOriented = true; // Workaround, change if needed

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate active subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2024_COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));

          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0));

          // algae = new AlgaeEffector(new AlgaeEffectorIOPhysical());
        }

        case ROBOT_2025_TEST -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));

          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0));
        }
        case ROBOT_2025_COMP -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonSpark(0),
                  new ModuleIOTalonSpark(1),
                  new ModuleIOTalonSpark(2),
                  new ModuleIOTalonSpark(3));
          coral = new CoralEffector(new CoralEffectorIOSparkMAX());
          climber = new Climber(new ClimberIOSparkMax());
          algae = new AlgaeEffector(new AlgaeEffectorIOPhysical());
          elevator = new Elevator(new ElevatorIOPhysical());
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());

          algae = new AlgaeEffector(new AlgaeEffectorIOSim());
          climber = new Climber(new ClimberIOSim());
        }
        case ROBOT_BRIEFCASE -> {
          algae = new AlgaeEffector(new AlgaeEffectorIOPhysical());
          // coral = new CoralEffector(new CoralEffectorIOSparkMAX());
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
    if (algae == null) {
      algae = new AlgaeEffector(new AlgaeEffectorIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (coral == null) {
      coral = new CoralEffector(new CoralEffectorIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    fieldAlignment = new FieldAlignment(drive);
    orchestrator = new Orchestrator(elevator, algae, coral);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

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

    coral.setDefaultCommand(coral.stop());
    algae.setDefaultCommand(algae.stowArm());
    climber.setDefaultCommand(climber.stop());
    elevator.setDefaultCommand(elevator.hold(elevator.getPosition()));
    driverController.y().onTrue(Commands.runOnce(() -> isRobotOriented = !isRobotOriented));
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    operatorController.x().onTrue(orchestrator.moveElevatorToLevel(false, 1));
    operatorController.y().onTrue(orchestrator.moveElevatorToLevel(false, 2));
    operatorController.a().onTrue(orchestrator.moveElevatorToLevel(false, 3));
    operatorController.b().onTrue(orchestrator.moveElevatorToLevel(false, 4));
    operatorController.leftTrigger().onTrue(orchestrator.removeAlgae(2));
    operatorController.leftBumper().onTrue(orchestrator.removeAlgae(3));
    operatorController.rightBumper().onTrue(climber.deploy());
    operatorController.rightTrigger().onTrue(climber.winch());
    driverController.leftBumper().onTrue(fieldAlignment.alignToReef(true));
    driverController.rightBumper().onTrue(fieldAlignment.alignToReef(false));
    driverController
        .leftTrigger()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    fieldAlignment.faceReef(driverController::getLeftX, driverController::getLeftY),
                    orchestrator.intake()),
                fieldAlignment.faceCoralStation(
                    driverController::getLeftX, driverController::getLeftY),
                coral::hasCoral));
    driverController.b().whileTrue(elevator.runPercent(0.3));
    driverController.y().whileTrue(elevator.runPercent(-0.3));
    driverController
        .a()
        .onTrue(
            Commands.either(
                coral.dumpCoral(),
                orchestrator.scoreL1(),
                () -> robotState.elevatorPosition != ElevatorPosition.L1));
    driverController.x().whileTrue(algae.removeAlgae());
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
    fieldAlignment.periodic();
  }
}
