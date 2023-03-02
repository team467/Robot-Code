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
import edu.wpi.first.math.util.Units;
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
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.commands.arm.ArmCalibrateCMD;
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
import frc.robot.commands.auto.AlignToNode;
import frc.robot.commands.auto.ScoreConeHigh;
import frc.robot.commands.auto.better.Leave;
import frc.robot.commands.auto.better.LeaveStraight6;
import frc.robot.commands.auto.better.ScoreOneLeave;
import frc.robot.commands.auto.better.ScoreOneLeaveBalance;
import frc.robot.commands.auto.better.StraightBack;
import frc.robot.commands.drive.DriveWithDpad;
import frc.robot.commands.drive.DriveWithJoysticks;
import frc.robot.commands.drive.GoToTrajectory;
import frc.robot.commands.intakerelease.HoldCMD;
import frc.robot.commands.intakerelease.IntakeCMD;
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
import frc.robot.subsystems.intakerelease.IntakeReleaseIOPhysical;
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
        // Init subsystems
        // subsystem = new Subsystem(new SubsystemIOImpl());
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
    //    autoChooser.addOption("Forward 1 meter", new GoToDistanceAngle(drive, 1.0, new
    // Rotation2d()));

    autoChooser.addOption(
        "Cross Community",
        new GoToTrajectory(
            drive,
            List.of(
                Waypoint.fromHolonomicPose(new Pose2d()),
                new Waypoint(new Translation2d(FieldConstants.Community.outerX, 0)))));
    autoChooser.addOption(
        "Score cone high", new ScoreConeHigh(drive, arm, intakeRelease, led2023, 6));
    //    autoChooser.addOption("Forward 1 meter", new GoToDistanceAngle(drive, 1.0, new
    // Rotation2d()));
    //        autoChooser.addOption("Drive Then Balance", new DriveFowardBallance(drive));
    //        autoChooser.addOption("Drive Foward and Come Back", new DriveFowardComeBack(drive));
    //        autoChooser.addOption(
    //            "Score then Drive Foward, then ballance", new ScoreDriveFowardBallance(drive));
    //    autoChooser.addOption(
    //        "Score Then Move", new ScoreThenMoveOut8(drive, arm, intakeRelease, led2023));

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
    autoChooser.addOption("Go to node", new AlignToNode(drive, () -> 1));
    autoChooser.addOption("Straight Back", new StraightBack(drive, arm, led2023));
    autoChooser.addOption("Leave", new Leave(drive, arm, led2023));
    autoChooser.addOption("Leave Straight", new LeaveStraight6(drive, arm, led2023));
    autoChooser.addOption(
        "Score one and leave", new ScoreOneLeave(drive, arm, intakeRelease, led2023));
    autoChooser.addOption(
        "Score one, leave and balance",
        new ScoreOneLeaveBalance(drive, arm, intakeRelease, led2023));
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
                                new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));
    driverController
        .pov(-1)
        .whileFalse(new DriveWithDpad(drive, () -> driverController.getHID().getPOV()));

    led2023.setDefaultCommand(new LedRainbowCMD(led2023).ignoringDisable(true));
    intakeRelease.setDefaultCommand(new HoldCMD(intakeRelease, led2023));

    driverController.leftBumper().whileTrue(new IntakeCMD(intakeRelease, led2023, arm));
    driverController.rightBumper().whileTrue(new ReleaseCMD(intakeRelease, led2023, arm));

    // Set the game piece type
    operatorController.back().onFalse(new WantConeCMD(intakeRelease, led2023));
    operatorController.back().onTrue(new WantCubeCMD(intakeRelease, led2023));

    // Manual arm movements
    operatorController.pov(90).whileTrue(new ArmManualExtendCMD(arm, intakeRelease, led2023));
    operatorController.pov(270).whileTrue(new ArmManualRetractCMD(arm, intakeRelease, led2023));
    operatorController.pov(180).whileTrue(new ArmManualDownCMD(arm, intakeRelease, led2023));
    operatorController.pov(0).whileTrue(new ArmManualUpCMD(arm, intakeRelease, led2023));

    // Placing cone or cube, gets what it wants from in the command
    operatorController.a().onTrue(new ArmScoreLowNodeCMD(arm, intakeRelease, led2023));
    operatorController.b().onTrue(new ArmScoreMidNodeCMD(arm, intakeRelease, led2023));
    operatorController.y().onTrue(new ArmScoreHighNodeCMD(arm, intakeRelease, led2023));
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
    driverController.back().onTrue(new ArmCalibrateCMD(arm, led2023));

    // Manual arm movements
    operatorController.leftStick().onTrue(new ArmStopCMD(arm, led2023));
    operatorController.rightStick().onTrue(new ArmStopCMD(arm, led2023));
    operatorController.leftBumper().onTrue(new ArmShelfCMD(arm, led2023));
    operatorController.rightBumper().onTrue(new ArmFloorCMD(arm, led2023));
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
      new WantCubeCMD(intakeRelease, led2023).schedule();
    } else {
      new WantConeCMD(intakeRelease, led2023).schedule();
    }
    Logger.getInstance()
        .recordOutput("CustomController/WantSwitch", operatorController.back().getAsBoolean());
  }
}