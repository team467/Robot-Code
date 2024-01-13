// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.characterization.FeedForwardCharacterization;
import frc.lib.characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.lib.io.gyro3d.IMUIO;
import frc.lib.io.gyro3d.IMUPigeon2;
import frc.lib.io.vision.Vision;
import frc.lib.io.vision.VisionIOPhotonVision;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.led.Leds;
import java.util.List;
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
  private boolean isRobotOriented = true; // Workaround, change if needed

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.get().mode()) {
        // Real robot, instantiate hardware IO implementations
      case REAL -> {
        switch (RobotConstants.get().robot()) {
          case ROBOT_COMP -> {
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
                    List.of(new VisionIOPhotonVision("front"), new VisionIOPhotonVision("right")),
                    List.of(front, right));
            drive =
                new Drive(
                    new IMUPigeon2(17),
                    new ModuleIOSparkMAX(3, 4, 13, 0),
                    new ModuleIOSparkMAX(5, 6, 14, 1),
                    new ModuleIOSparkMAX(1, 2, 15, 2),
                    new ModuleIOSparkMAX(7, 8, 16, 3));
          }
          case ROBOT_BRIEFCASE -> {
            drive =
                new Drive(
                    new IMUIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
          }
          default -> {
            drive =
                new Drive(
                    new IMUIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
          }
        }
      }
        // Sim robot, instantiate physics sim IO implementations
      case SIM -> {
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOSim());
        drive =
            new Drive(
                new IMUIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
      }

        // Replayed robot, disable IO implementations
      default -> {
        // subsystem = new Subsystem(new SubsystemIO() {});
        drive =
            new Drive(
                new IMUIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
      }
    }

    Leds.getInstance();
    // led2023 = new Led2023();
    // LEDManager.getInstance().init(RobotConstants.get().ledChannel());

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
                    drive::runDriveCharacterizationVolts,
                    drive::getDriveCharacterizationVelocity))
            .andThen(() -> configureButtonBindings()));

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
            () -> -driverController.getRightX(),
            () -> isRobotOriented // TODO: add toggle
            ));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    // led2023.setDefaultCommand(new LedRainbowCMD(led2023).ignoringDisable(true));
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
