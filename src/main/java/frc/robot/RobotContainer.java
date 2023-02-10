// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.characterization.FeedForwardCharacterization;
import frc.lib.characterization.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.lib.holonomictrajectory.Waypoint;
import frc.lib.io.gyro3d.IMUIO;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.arm.ArmManualExtendCMD;
import frc.robot.commands.arm.ArmManualRetractCMD;
import frc.robot.commands.arm.ArmScoreCMD;
import frc.robot.commands.arm.ArmStopCMD;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOPhysical;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
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
  private final Drive drive;
  private final Arm arm;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (RobotConstants.get().mode()) {
        // Real robot, instantiate hardware IO implementations
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOImpl());
      case REAL -> {
        switch (RobotConstants.get().robot()) {
          case ROBOT_COMP -> {
            drive =
                new Drive(
                    new IMUIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
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

    arm =
        new Arm(
            new ArmIOPhysical(
                RobotConstants.get().armExtendMotorId(),
                RobotConstants.get().armRotateMotorId(),
                RobotConstants.get().extendLimitSwitchId()));

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption(
        "S shape",
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(new Pose2d()),
                new Waypoint(new Translation2d(1, 1)),
                new Waypoint(new Translation2d(2, -1)),
                Waypoint.fromHolonomicPose(new Pose2d(3, 0, Rotation2d.fromDegrees(90))))));
    //    autoChooser.addOption("Forward 1 meter", new GoToDistanceAngle(drive, 1.0, new
    // Rotation2d()));
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
            .andThen(() -> configureButtonBindings()));
    // autoChooser.addOption("AutoCommand", new AutoCommand(subsystem));

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
        new DriveWithJoysticks(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true // TODO: add toggle
            ));
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));
    driverController.leftBumper().onTrue(new ArmManualExtendCMD(arm));
    driverController.start().onTrue(new ArmStopCMD(arm));
    driverController.rightBumper().onTrue(new ArmManualRetractCMD(arm));
    driverController.leftTrigger().onTrue(new ArmScoreCMD(arm, 0.85)); // Extend full
    driverController.rightTrigger().onTrue(new ArmScoreCMD(arm, 0.75)); // Extend Mid
    driverController.a().onTrue(new ArmScoreCMD(arm, 0.5)); // Retract full
    driverController.b().onTrue(new ArmScoreCMD(arm, 0.6)); // Retract Mid
    driverController.y().onTrue(new ArmScoreCMD(arm, 0.65));
    driverController.x().onTrue(new ArmScoreCMD(arm, 0.7));
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
