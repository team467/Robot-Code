// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.lib.io.gyro3d.IMUPigeon2;
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOAprilTag;
import frc.lib.leds.LEDManager;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.arm.ArmManualDownCMD;
import frc.robot.commands.arm.ArmManualExtendCMD;
import frc.robot.commands.arm.ArmManualRetractCMD;
import frc.robot.commands.arm.ArmManualUpCMD;
import frc.robot.commands.arm.ArmRetractCMD;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreLowNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.arm.ArmStopCMD;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.commands.intakerelease.HoldCMD;
import frc.robot.commands.intakerelease.IntakeCMD;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantConeCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.commands.leds.LedRainbowCMD;
import frc.robot.commands.leds.LedResetPoseCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOPhysical;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeReleaseIO;
import frc.robot.subsystems.intakerelease.IntakeReleaseIOPhysical;
import frc.robot.subsystems.led.Led2023;
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
  private final IntakeRelease intakeRelease;
  private Led2023 led2023;
  private final Arm arm;

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
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOImpl());
      case REAL -> {
        switch (RobotConstants.get().robot()) {
          case ROBOT_COMP -> {
            Transform3d front =
                new Transform3d(
                    new Translation3d(6 * 0.01, -10 * 0.01, 42 * 0.01), new Rotation3d());
            Transform3d right =
                new Transform3d(
                    new Translation3d(2 * 0.01, -12 * 0.01, 42 * 0.01),
                    new Rotation3d(0, 0, -0.5 * Math.PI));
            drive =
                new Drive(
                    new IMUPigeon2(17),
                    new ModuleIOSparkMAX(3, 4, 13, 0),
                    new ModuleIOSparkMAX(5, 6, 14, 1),
                    new ModuleIOSparkMAX(1, 2, 15, 2),
                    new ModuleIOSparkMAX(7, 8, 16, 3),
                    List.of(
                        new VisionIOAprilTag("front", front, FieldConstants.aprilTagFieldLayout),
                        new VisionIOAprilTag("right", right, FieldConstants.aprilTagFieldLayout)));
            arm =
                new Arm(
                    new ArmIOPhysical(
                        RobotConstants.get().armExtendMotorId(),
                        RobotConstants.get().armRotateMotorId(),
                        RobotConstants.get().ratchetSolenoidId()));
            intakeRelease =
                new IntakeRelease(
                    new IntakeReleaseIOPhysical(
                        RobotConstants.get().intakeMotorID(),
                        RobotConstants.get().intakeCubeLimitSwitchID()));
          }
          case ROBOT_BRIEFCASE -> {
            drive =
                new Drive(
                    new IMUIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    List.of(new VisionIO() {}));
            arm = new Arm(new ArmIO() {});
            intakeRelease = new IntakeRelease(new IntakeReleaseIO() {});
          }
          default -> {
            drive =
                new Drive(
                    new IMUIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    List.of(new VisionIO() {}));
            arm = new Arm(new ArmIO() {});
            intakeRelease = new IntakeRelease(new IntakeReleaseIO() {});
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
                new ModuleIOSim(),
                List.of(new VisionIO() {}));
        arm = new Arm(new ArmIO() {});
        intakeRelease = new IntakeRelease(new IntakeReleaseIO() {});
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
                new ModuleIO() {},
                List.of(new VisionIO() {}));
        arm = new Arm(new ArmIO() {});
        intakeRelease = new IntakeRelease(new IntakeReleaseIO() {});
      }
    }

    led2023 = new Led2023();
    LEDManager.getInstance().init(RobotConstants.get().ledChannel());

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
    // autoChooser.addOption("Forward 1 meter", new GoToDistanceAngle(drive, 1.0,
    new Rotation2d();
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
    driverController.start().onTrue(new LedResetPoseCMD(led2023, drive));
    led2023.setDefaultCommand(new LedRainbowCMD(led2023, intakeRelease).ignoringDisable(true));
    intakeRelease.setDefaultCommand(new HoldCMD(intakeRelease, led2023));

    switch (RobotConstants.get().mode()) {
      case REAL -> {
        driverController.leftBumper().whileTrue(new IntakeCMD(intakeRelease, led2023));
        driverController.rightBumper().whileTrue(new ReleaseCMD(intakeRelease, led2023));
        operatorController.back().toggleOnTrue(new WantConeCMD(intakeRelease, led2023));
        operatorController.start().toggleOnTrue(new WantCubeCMD(intakeRelease, led2023));

        operatorController.start().onTrue(new ArmStopCMD(arm));
        operatorController.pov(90).whileTrue(new ArmManualExtendCMD(arm, intakeRelease, led2023));
        operatorController.pov(270).whileTrue(new ArmManualRetractCMD(arm, intakeRelease, led2023));
        operatorController.pov(180).whileTrue(new ArmManualDownCMD(arm, intakeRelease, led2023));
        operatorController.pov(0).whileTrue(new ArmManualUpCMD(arm, intakeRelease, led2023));
        operatorController.x().onTrue(new ArmHomeCMD(arm)); // Retract full
        operatorController.a().onTrue(new ArmScoreLowNodeCMD(arm));
        operatorController.b().onTrue(new ArmScoreMidNodeCMD(arm));
        operatorController.y().onTrue(new ArmScoreHighNodeCMD(arm));
        operatorController.back().onTrue(new ArmRetractCMD(arm));
        operatorController.rightTrigger().onTrue(new ArmCalibrateCMD(arm, led2023));
      }
      case REPLAY -> {}
      case SIM -> {}
    }
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
