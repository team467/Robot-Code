// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.characterization.FeedForwardCharacterization;
import frc.lib.characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.lib.io.gyro3d.GyroIO;
import frc.lib.io.gyro3d.GyroPigeon2;
import frc.lib.io.vision.Vision;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMAX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOPhysical;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.pixy2.Pixy2;
import frc.robot.subsystems.pixy2.Pixy2IO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOPhysical;
import java.util.List;
import java.util.Set;
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
  private Shooter shooter;
  private Indexer indexer;
  private Intake intake;
  private Drive drive;
  private Arm arm;
  private Vision vision;
  private Pixy2 pixy2;
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
        case ROBOT_2023 -> {
          Transform3d front =
              new Transform3d(
                  new Translation3d(6 * 0.01, -10 * 0.01 - Units.inchesToMeters(2.0), 42 * 0.01),
                  new Rotation3d());
          Transform3d right =
              new Transform3d(
                  new Translation3d(2 * 0.01, -12 * 0.01 - Units.inchesToMeters(2.0), 42 * 0.01),
                  new Rotation3d(0, 0, -0.5 * Math.PI));
          vision =
              new Vision(
                  List.of(
                          new VisionIOPhotonVision("front", front),
                          new VisionIOPhotonVision("right", right))
                      .toArray(new frc.lib.io.vision.VisionIO[0]));
          drive =
              new Drive(
                  new GyroPigeon2(Schematic.GYRO_ID),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
        }
        case ROBOT_2024C -> {
          drive =
              new Drive(
                  new GyroPigeon2(Schematic.GYRO_ID),
                  new ModuleIOSparkMAX(0),
                  new ModuleIOSparkMAX(1),
                  new ModuleIOSparkMAX(2),
                  new ModuleIOSparkMAX(3));
          arm = new Arm(new ArmIOSparkMAX());
          indexer = new Indexer(new IndexerIOPhysical());
          intake = new Intake(new IntakeIOPhysical());
          shooter = new Shooter(new ShooterIOPhysical());
        }

        case ROBOT_SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
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
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (pixy2 == null) {
      pixy2 = new Pixy2(new Pixy2IO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }

    Leds leds = new Leds();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", Commands.none());

    autoChooser.addOption(
        "Drive Characterization",
        Commands.runOnce(() -> drive.setPose(new Pose2d()), drive)
            .andThen(
                new FeedForwardCharacterization(
                    drive,
                    true,
                    new FeedForwardCharacterizationData("drive"),
                    drive::runCharacterizationVolts,
                    drive::getCharacterizationVelocity))
            .andThen(this::configureButtonBindings));

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
    driverController.y().onTrue(Commands.runOnce(() -> isRobotOriented = !isRobotOriented));
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> isRobotOriented // TODO: add toggle
            ));
    driverController
        .start()
        .onTrue(
            Commands.defer(
                    () ->
                        Commands.runOnce(
                            () ->
                                drive.setPose(
                                    new Pose2d(
                                        drive.getPose().getTranslation(),
                                        AllianceFlipUtil.apply(new Rotation2d())))),
                    Set.of())
                .ignoringDisable(true));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    // stop when doing nothing
    intake.setDefaultCommand(intake.stop());
    indexer.setDefaultCommand(indexer.setPercent(0));
    shooter.setDefaultCommand(shooter.manualShoot(0));
    arm.setDefaultCommand(arm.hold());

    // operator controller
    operatorController
        .leftBumper()
        .and(() -> !indexer.getLimitSwitchPressed())
        .whileTrue(
            (intake.intake().alongWith(indexer.setPercent(IndexerConstants.INDEX_SPEED.get())))
                .onlyWhile(() -> !indexer.getLimitSwitchPressed())
                .andThen(indexer.setPercent(IndexerConstants.INDEX_SPEED.get()).withTimeout(0.2)));

    operatorController.y().whileTrue(indexer.setPercent(1));

    operatorController.b().whileTrue(indexer.setPercent(-0.8).alongWith(intake.release()));
    operatorController.rightBumper().whileTrue(shooter.manualShoot(-0.2 * 12));
    operatorController.a().whileTrue(shooter.manualShoot(10));

    // operator d pad
    operatorController.pov(0).whileTrue(arm.runPercent(0.2));
    operatorController.pov(180).whileTrue(arm.runPercent(-0.2));
    operatorController.pov(90).whileTrue(arm.runPercent(0));

    driverController.rightBumper().whileTrue(arm.toSetpoint(ArmConstants.OFFSET));
    driverController.leftBumper().whileTrue(arm.toSetpoint(Rotation2d.fromDegrees(78.26)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
