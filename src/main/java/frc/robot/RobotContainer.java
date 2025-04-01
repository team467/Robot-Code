// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutosAlternate;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.FieldAlignment;
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
import frc.robot.subsystems.fastalgae.FastAlgaeEffector;
import frc.robot.subsystems.fastalgae.FastAlgaeEffectorIO;
import frc.robot.subsystems.fastalgae.FastAlgaeEffectorIOPhysical;
import frc.robot.subsystems.leds.Leds;
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
  private FastAlgaeEffector fastalgae;
  private CoralEffector coral;
  private Climber climber;
  private Elevator elevator;
  private Leds leds;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;
  private RobotState robotState = RobotState.getInstance();
  private boolean isRobotOriented = true; // Workaround, change if needed

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final AutoRoutines autoRoutines;

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
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
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
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));
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
          fastalgae = new FastAlgaeEffector(new FastAlgaeEffectorIOPhysical());
          elevator = new Elevator(new ElevatorIOPhysical());
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));
          leds = new Leds();
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          climber = new Climber(new ClimberIOSim());

          leds = new Leds();
        }

        case ROBOT_BRIEFCASE -> {
          leds = new Leds();

          //           coral = new CoralEffector(new CoralEffectorIOSparkMAX());
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
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (coral == null) {
      coral = new CoralEffector(new CoralEffectorIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (fastalgae == null) {
      fastalgae = new FastAlgaeEffector(new FastAlgaeEffectorIO() {});
    }
    fieldAlignment = new FieldAlignment(drive);
    orchestrator = new Orchestrator(elevator, fastalgae, coral, drive);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoRoutines = new AutoRoutines(drive);

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

    AutosAlternate autosAlternate =
        new AutosAlternate(drive, orchestrator, fieldAlignment, coral, elevator);
    autoChooser.addOption("Zero Piece", autosAlternate.zeroPiece());
    autoChooser.addOption("A Score Left", autosAlternate.AScore(true));
    autoChooser.addOption("A Score Right", autosAlternate.AScore(false));
    autoChooser.addOption("B Score Left", autosAlternate.BScore(true));
    autoChooser.addOption("B Score Right", autosAlternate.BScore(false));
    autoChooser.addOption("C Score Left", autosAlternate.CScore(true));
    autoChooser.addOption("C Score Right", autosAlternate.CScore(false));
    autoChooser.addOption("B Score Hope and Pray", autosAlternate.BScoreHopeAndPray());
    autoChooser.addOption("A Sigma Two Score", autosAlternate.sigmaATwoScore(false));
    autoChooser.addOption("A Alpha Three Score", autosAlternate.alphaAThreeScore(false));
    autoChooser.addOption(
        "A Omega Three Point Five Score", autosAlternate.omegaAThreePointFiveScore(false));
    autoChooser.addOption("A Skibidi Four Score", autosAlternate.skibidiAFourScore(false));
    autoChooser.addOption("C Sigma Two Score", autosAlternate.sigmaCTwoScore(true));
    autoChooser.addOption("C Alpha Three Score", autosAlternate.alphaCThreeScore(true));
    autoChooser.addOption(
        "C Omega Three Point Five Score", autosAlternate.omegaCThreePointFiveScore(false));
    autoChooser.addOption("C Skibidi Four Score", autosAlternate.skibidiCFourScore(false));
    autoChooser.addOption("Elevator Test", autosAlternate.elevatorRelativeToPose(true, 4));
    autoChooser.addOption("C6-2 Coral", autosAlternate.C6Mpath2Coral());
    autoChooser.addOption("A2-2 Coral", autosAlternate.A2Mpath2Coral());
    registerAutoRoutines();

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
    fastalgae.setDefaultCommand(fastalgae.stowArm());
    elevator.setDefaultCommand(elevator.runPercent(0.0));
    climber.setDefaultCommand(climber.stop());
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
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    CustomTriggers.autoModeInput(
            new Trigger(() -> driverController.getHID().getPOV() != -1), new Trigger(() -> false))
        .whileTrue(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));
    CustomTriggers.manualModeInput(
            new Trigger(() -> driverController.getHID().getPOV() != -1), new Trigger(() -> false))
        .whileTrue(
            fieldAlignment.updateMidMatchTunableOffsets(() -> driverController.getHID().getPOV()));
    CustomTriggers.autoModeInput(operatorController.x(), operatorController.back())
        .onTrue(orchestrator.moveElevatorToLevel(1));
    CustomTriggers.autoModeInput(operatorController.y(), operatorController.back())
        .onTrue(orchestrator.moveElevatorToLevel(2));
    CustomTriggers.autoModeInput(operatorController.a(), operatorController.back())
        .onTrue(orchestrator.moveElevatorToLevel(3));
    CustomTriggers.manualModeInput(operatorController.a(), operatorController.back())
        .whileTrue(elevator.runPercent(-0.3));
    CustomTriggers.autoModeInput(operatorController.a(), operatorController.back())
        .onTrue(orchestrator.moveElevatorToLevel(3));
    CustomTriggers.manualModeInput(operatorController.b(), operatorController.back())
        .whileTrue(elevator.runPercent(0.3));
    CustomTriggers.autoModeInput(operatorController.b(), operatorController.back())
        .onTrue(orchestrator.moveElevatorToLevel(4));
    CustomTriggers.autoModeInput(operatorController.leftTrigger(), operatorController.back())
        .toggleOnTrue(orchestrator.removeAlgae(2));
    CustomTriggers.autoModeInput(operatorController.leftBumper(), operatorController.back())
        .toggleOnTrue(orchestrator.removeAlgae(3));
    CustomTriggers.autoModeInput(operatorController.pov(0), operatorController.back())
        .whileTrue(climber.deploy());
    CustomTriggers.autoModeInput(operatorController.pov(180), operatorController.back())
        .whileTrue(climber.winch());
    CustomTriggers.manualModeInput(operatorController.rightBumper(), operatorController.back())
        .whileTrue(climber.runPercent(0.15));
    CustomTriggers.manualModeInput(operatorController.rightTrigger(), operatorController.back())
        .whileTrue(climber.runPercent(-0.15));
    driverController.leftBumper().toggleOnTrue(fieldAlignment.alignToReefMatchTunable(true));
    driverController.rightBumper().toggleOnTrue(fieldAlignment.alignToReefMatchTunable(false));
    CustomTriggers.autoModeInput(driverController.leftTrigger(), operatorController.rightTrigger())
        .toggleOnTrue(
            Commands.parallel(
                fieldAlignment.faceCoralStation(driverController::getLeftX, driverController::getLeftY),
                orchestrator.intake(),
                Commands.startEnd(
                    () -> driverController.setRumble(RumbleType.kBothRumble, 0.5),
                    () -> driverController.setRumble(RumbleType.kBothRumble, 0.0)
                )
            ).until(coral::hasCoral)
        );
    CustomTriggers.manualModeInput(driverController.leftTrigger(), new Trigger(() -> false))
        .toggleOnTrue(orchestrator.intake());
    driverController
        .a()
        .toggleOnTrue(
            fieldAlignment.faceReef(driverController::getLeftX, driverController::getLeftY));
    driverController.x().whileTrue(coral.takeBackCoral());
    driverController.rightTrigger(0.1).onTrue(orchestrator.dumpCoralAndHome());
    driverController.rightTrigger(0.1).onTrue(drive.runOnce(Commands::none));
    driverController.y().whileTrue(elevator.runPercent(-0.3));
  }

  private void addAutoRoutine(String routineName) {
    autoChooser.addOption(routineName, autoRoutines.getRoutines().get(routineName).cmd());
  }

  private void registerAutoRoutines() {
    addAutoRoutine("A leave");
    addAutoRoutine("C6L5RL");
    addAutoRoutine("C5RL4R");
    addAutoRoutine("B1R2LR");
    addAutoRoutine("B1L6RL");
    addAutoRoutine("B1R");
    addAutoRoutine("B1L");
    addAutoRoutine("A3LR4L");
    addAutoRoutine("A2R3LR");
    addAutoRoutine("C leave");
    addAutoRoutine("B leave");
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
    RobotState.getInstance().updateLEDState();
  }
}
