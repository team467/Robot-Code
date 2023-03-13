// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.lib.io.vision.VisionIO;
import frc.lib.io.vision.VisionIOAprilTag;
import frc.lib.leds.LEDManager;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.arm.ArmCalibrateCMD;
import frc.robot.commands.arm.ArmCalibrateZeroAtHomeCMD;
import frc.robot.commands.arm.ArmFloorCMD;
import frc.robot.commands.arm.ArmHomeCMD;
import frc.robot.commands.arm.ArmManualDownCMD;
import frc.robot.commands.arm.ArmManualExtendCMD;
import frc.robot.commands.arm.ArmManualRetractCMD;
import frc.robot.commands.arm.ArmManualUpCMD;
import frc.robot.commands.arm.ArmScoreHighNodeCMD;
import frc.robot.commands.arm.ArmScoreLowNodeCMD;
import frc.robot.commands.arm.ArmScoreMidNodeCMD;
import frc.robot.commands.arm.ArmShelfCMD;
import frc.robot.commands.arm.ArmStopCMD;
import frc.robot.commands.auto.complex.BackUpAndBalance;
import frc.robot.commands.auto.complex.OnlyBackup;
import frc.robot.commands.auto.complex.OnlyBalance;
import frc.robot.commands.auto.complex.OnlyScore;
import frc.robot.commands.auto.complex.ScoreAndBackUp;
import frc.robot.commands.auto.complex.ScoreAndBackUpAndBalance;
import frc.robot.commands.auto.complex.ScoreAndBalance;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.intakerelease.HoldCMD;
import frc.robot.commands.intakerelease.IntakeAndRaise;
import frc.robot.commands.intakerelease.ReleaseCMD;
import frc.robot.commands.intakerelease.WantConeCMD;
import frc.robot.commands.intakerelease.WantCubeCMD;
import frc.robot.commands.leds.LedRainbowCMD;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOPhysical;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.subsystems.intakerelease.IntakeRelease;
import frc.robot.subsystems.intakerelease.IntakeReleaseIO;
import frc.robot.subsystems.intakerelease.IntakeReleaseIOBrushed;
import frc.robot.subsystems.led.Led2023;
import java.util.List;
import org.littletonrobotics.junction.Logger;
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
            drive =
                new Drive(
                    new IMUPigeon2(17),
                    new ModuleIOSparkMAX(3, 4, 13, 0),
                    new ModuleIOSparkMAX(5, 6, 14, 1),
                    new ModuleIOSparkMAX(1, 2, 15, 2),
                    new ModuleIOSparkMAX(7, 8, 16, 3),
                    List.of(
                        new VisionIOAprilTag("front", front, FieldConstants.aprilTags),
                        new VisionIOAprilTag("right", right, FieldConstants.aprilTags)));
            arm =
                new Arm(
                    new ArmIOPhysical(
                        RobotConstants.get().armExtendMotorId(),
                        RobotConstants.get().armRotateMotorId(),
                        RobotConstants.get().ratchetSolenoidId()));
            intakeRelease =
                new IntakeRelease(
                    new IntakeReleaseIOBrushed(
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

    led2023 = new Led2023(arm, intakeRelease);
    LEDManager.getInstance().init(RobotConstants.get().ledChannel());

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new ArmCalibrateZeroAtHomeCMD(arm));

    // AprilTag 3 or 6
    autoChooser.addOption(
        "Tag 3/6: Only Back Up", new OnlyBackup(6, "Right", drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Tag 3/6: Only Score Cone",
        new OnlyScore(6, "Right", "Cone", "High", drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Tag 3/6: Score Cone and Back Up",
        new ScoreAndBackUp(6, "Right", "Cone", "High", drive, arm, intakeRelease, led2023));

    // AprilTag 2 or 7
    autoChooser.addOption(
        "Tag 2/7: Only Score Cone",
        new OnlyScore(7, "Right", "cone", "high", drive, arm, intakeRelease, led2023));
    autoChooser.addOption("Tag 2/7: Only Balance", new OnlyBalance("Right", drive, arm, led2023));
    autoChooser.addOption(
        "Tag 2/7: Back Up and Balance", new BackUpAndBalance("Center", drive, arm, led2023));
    autoChooser.addOption(
        "Tag 2/7: Score and Balance",
        new ScoreAndBalance("Right", "Cone", "High", drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Tag 2/7: Score, Back Up and Balance",
        new ScoreAndBackUpAndBalance("Right", "Cone", "High", drive, arm, intakeRelease, led2023));

    // AprilTag 1 or 8
    autoChooser.addOption(
        "Tag 1/8: Only Back Up", new OnlyBackup(8, "Left", drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Tag 1/8: Only Score Cone",
        new OnlyScore(8, "Left", "Cone", "High", drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Tag 1/8: Score Cone and Back Up",
        new ScoreAndBackUp(8, "Left", "Cone", "High", drive, arm, intakeRelease, led2023));

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

    led2023.setDefaultCommand(new LedRainbowCMD(led2023).ignoringDisable(true));
    intakeRelease.setDefaultCommand(new HoldCMD(intakeRelease));

    driverController.leftBumper().toggleOnTrue(new IntakeAndRaise(arm, intakeRelease));
    driverController.rightBumper().toggleOnTrue(new ReleaseCMD(intakeRelease, arm));

    // Set the game piece type
    operatorController.back().whileFalse(new WantConeCMD(intakeRelease));
    operatorController.back().whileTrue(new WantCubeCMD(intakeRelease));

    // Manual arm movements
    operatorController.pov(90).whileTrue(new ArmManualExtendCMD(arm));
    operatorController.pov(270).whileTrue(new ArmManualRetractCMD(arm));
    operatorController.pov(180).whileTrue(new ArmManualDownCMD(arm));
    operatorController.pov(0).whileTrue(new ArmManualUpCMD(arm));

    // Placing cone or cube, gets what it wants from in the command
    operatorController.a().onTrue(new ArmScoreLowNodeCMD(arm, intakeRelease));
    operatorController.b().onTrue(new ArmScoreMidNodeCMD(arm, intakeRelease));
    operatorController.y().onTrue(new ArmScoreHighNodeCMD(arm, intakeRelease));
    Logger.getInstance()
        .recordOutput("CustomController/LowButton", operatorController.a().getAsBoolean());
    Logger.getInstance()
        .recordOutput("CustomController/MiddleButton", operatorController.b().getAsBoolean());
    Logger.getInstance()
        .recordOutput("CustomController/HighButton", operatorController.y().getAsBoolean());
    Logger.getInstance()
        .recordOutput("CustomController/HomeButton", operatorController.x().getAsBoolean());

    // Home will be for movement
    operatorController.x().onTrue(new ArmHomeCMD(arm, led2023));
    driverController.x().onTrue(new ArmHomeCMD(arm, led2023));

    // Need to set to use automated movements, should be set in Autonomous init.
    driverController.back().onTrue(new ArmCalibrateCMD(arm));
    driverController.b().onTrue(new ArmCalibrateZeroAtHomeCMD(arm));

    driverController.a().onTrue(Commands.runOnce(() -> drive.stopWithX(), drive));

    // Manual arm movements
    operatorController.leftStick().onTrue(new ArmStopCMD(arm));
    operatorController.rightStick().onTrue(new ArmStopCMD(arm));
    operatorController.leftBumper().onTrue(new ArmShelfCMD(arm, intakeRelease));
    operatorController.rightBumper().onTrue(new ArmFloorCMD(arm, intakeRelease));
    Logger.getInstance()
        .recordOutput(
            "CustomController/FloorButton", operatorController.rightBumper().getAsBoolean());
    Logger.getInstance()
        .recordOutput(
            "CustomController/ShelfButton", operatorController.leftBumper().getAsBoolean());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void initLeds() {
    // Set default LEDs
    if (operatorController.back().getAsBoolean()) {
      new WantCubeCMD(intakeRelease).schedule();
    } else {
      new WantConeCMD(intakeRelease).schedule();
    }
    Logger.getInstance()
        .recordOutput("CustomController/WantSwitch", operatorController.back().getAsBoolean());
  }
}
