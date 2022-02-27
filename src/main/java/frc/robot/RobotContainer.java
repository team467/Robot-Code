// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.controllers.CustomController2020;
import frc.robot.controllers.XboxController467;
import frc.robot.subsystems.Climber2020;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubCameraLED;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2020;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  HashMap<String, Trajectory> trajectories = new HashMap<>();

  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain = null;
  private Gyro gyro = null;
  private Climber2020 climber = null;
  private Shooter2020 shooter = null;
  private LlamaNeck2022 llamaNeck = null;
  private Indexer2022 indexer = null;
  private Spitter2022 spitter = null;
  private Shooter2022 shooter2022 = null;
  private HubCameraLED hubCameraLED = null;

  // User interface objects
  // Xbox controller for driver
  private final XboxController467 driverJoystick = new XboxController467(0);
  private final JoystickButton driverButtonA = new JoystickButton(driverJoystick, XboxController467.Buttons.A.value);
  private final JoystickButton driverButtonB = new JoystickButton(driverJoystick, XboxController467.Buttons.B.value);
  private final JoystickButton driverButtonX = new JoystickButton(driverJoystick, XboxController467.Buttons.X.value);
  private final JoystickButton driverButtonY = new JoystickButton(driverJoystick, XboxController467.Buttons.Y.value);
  private final JoystickButton driverButtonBack = new JoystickButton(driverJoystick,
      XboxController467.Buttons.Back.value);
  private final JoystickButton driverButtonStart = new JoystickButton(driverJoystick,
      XboxController467.Buttons.Start.value);
  private final JoystickButton driverPovUp = new JoystickButton(driverJoystick, XboxController467.Buttons.POVup.value);
  private final JoystickButton driverPovDown = new JoystickButton(driverJoystick,
      XboxController467.Buttons.POVdown.value);
  private final JoystickButton driverPovLeft = new JoystickButton(driverJoystick,
      XboxController467.Buttons.POVleft.value);
  private final JoystickButton driverPovRight = new JoystickButton(driverJoystick,
      XboxController467.Buttons.POVright.value);
  private final JoystickButton driverLeftBumper = new JoystickButton(driverJoystick,
      XboxController467.Buttons.BumperLeft.value);
  private final JoystickButton driverRightBumper = new JoystickButton(driverJoystick,
      XboxController467.Buttons.BumperRight.value);

  // Custom controller for operator
  private final GenericHID operatorJoystick = new Joystick(1);
  private final JoystickButton operatorInakeArm = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INTAKE_ARM.value);
  private final JoystickButton operatorIntakeRollerForward = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INTAKE_ROLLER_FORWARD.value);
  private final JoystickButton operatorIntakeRollerBackward = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INTAKE_ROLLER_BACKWARD.value);
  private final JoystickButton operatorIndexAuto = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INDEX_AUTO.value);
  private final JoystickButton operatorIndexRollerForward = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INDEX_ROLLER_FORWARD.value);
  private final JoystickButton operatorIndexRollerBackward = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.INDEX_ROLLER_BACKWARD.value);
  private final JoystickButton operatorShooterAuto = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.SHOOTER_AUTO.value);
  private final JoystickButton operatorShooterFlywheel = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.SHOOTER_FLYWHEEL.value);
  private final JoystickButton operatorShooterShoot = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.SHOOTER_SHOOT.value);
  private final JoystickButton operatorClimberLock = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.CLIMBER_LOCK_SWITCH.value);
  private final JoystickButton operatorClimberUp = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.CLIMBER_UP_BUTTON.value);
  private final JoystickButton operatorClimberDown = new JoystickButton(operatorJoystick,
      CustomController2020.Buttons.CLIMBER_DOWN_BUTTON.value);

  public RobotContainer() {
    getTrajectories();

    initializeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
  }

  public void getTrajectories() {
    for (File file : Objects.requireNonNull(Filesystem.getDeployDirectory().toPath().resolve("paths")
            .resolve(RobotConstants.get().name().toLowerCase().replace(" ", "")).resolve("output").toFile().listFiles())) {
      try {
        trajectories.put(file.getName().replace(".wpilib.json", ""), TrajectoryUtil.fromPathweaverJson(file.toPath()));
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Use this method to define your button to command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void initializeSubsystems() {
    initGyro();
    initDrivetrain();
    initClimber2020();
    initShooter2020();
    initIndexer2022();
    initSpitter2022();
    initLlamaNeck2022();
    initShooter2022();
    initHubCameraLED();
  }

  private void initGyro() {
    if (RobotConstants.get().hasGyro()) {
      gyro = new Gyro();
    }
  }

  public void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    configureDrivetrain();
    configureClimber2020();
    configureShooter2020();
    configureLlamaNeck2022();
    configureIndexer2022();
    configureSpitter2022();
    configureShooter2022();
    driverButtonX.whenPressed(getAutonomousCommand());
  }

  private void initDrivetrain() {
    if (RobotConstants.get().hasDrivetrain()) {
      drivetrain = new Drivetrain();
    }
  }

  private void configureDrivetrain() {
    if (RobotConstants.get().hasDrivetrain()) {
      drivetrain.setDefaultCommand(new ArcadeDriveCMD(drivetrain,
              driverJoystick::getAdjustedDriveSpeed,
              driverJoystick::getAdjustedTurnSpeed));
      // operatorShooterShoot.whileHeld(new PuppyModeCMD(drivetrain));
      // driverButtonX.whileHeld(new GoToBallCMD(drivetrain, gyro));
      driverButtonY.whileHeld(new GoToTargetCMD(drivetrain, gyro));
      // driverButtonA.whenPressed(new TurnAngleCMD(drivetrain, gyro, 90));
      // driverButtonY.whileHeld(new GoToTrajectoryCMD(drivetrain, gyro,
      // trajectories.get("Reverse")));
      // driverButtonA.whileHeld(new GoToDistanceAngleCMD(drivetrain, gyro, 2.0, 0.0,
      // true));
      // driverButtonA.whenPressed(new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(),
      //     new Pose2d(-2, 0, Rotation2d.fromDegrees(0)), true));
      // driverButtonB.whenPressed(new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(),
      //     new Pose2d(2, 0, Rotation2d.fromDegrees(0)), false));
    }
  }

  private void initClimber2020() {
    if (RobotConstants.get().hasClimber2020()) {
      climber = new Climber2020();
    }
  }

  private void configureClimber2020() {
    if (RobotConstants.get().hasClimber2020()) {
      climber.setDefaultCommand(new ClimberStopCMD(climber));
      operatorClimberLock.whenPressed(new ClimberEnableCMD(climber));
      operatorClimberUp.whenHeld(new ClimberUpCMD(climber));
      operatorClimberDown.whenHeld(new ClimberDownCMD(climber));
    }
  }

  private void initShooter2020() {
    if (RobotConstants.get().hasShooter2020()) {
      shooter = new Shooter2020();
    }
  }

  private void configureShooter2020() {
    if (RobotConstants.get().hasShooter2020()) {
      operatorShooterFlywheel.whenPressed(new ShooterRunFlywheelCMD(shooter));
      operatorShooterFlywheel.whenReleased(new ShooterStopFlywheelCMD(shooter));
      operatorShooterShoot.whenPressed(new ShooterTriggerForwardCMD(shooter));
      operatorShooterShoot.whenReleased(new ShooterTriggerStopCMD(shooter));
    }
  }

  private void initLlamaNeck2022() {
    if (RobotConstants.get().hasLlamaNeck2022()) {
      llamaNeck = new LlamaNeck2022();
    }
  }

  private void configureLlamaNeck2022() {
    if (RobotConstants.get().hasLlamaNeck2022()) {
      llamaNeck.setDefaultCommand(new LlamaNeck2022StopCMD(llamaNeck));
    }
  }

  private void initIndexer2022() {
    if (RobotConstants.get().hasIndexer2022()) {
      indexer = new Indexer2022();
    }
  }

  private void configureIndexer2022() {
    if (RobotConstants.get().hasIndexer2022()) {
      indexer.setDefaultCommand(new Indexer2022StopCMD(indexer));
    }
  }

  private void initSpitter2022() {
    if (RobotConstants.get().hasSpitter2022()) {
      spitter = new Spitter2022();
    }
  }

  private void configureSpitter2022() {
    if (RobotConstants.get().hasSpitter2022()) {
      spitter.setDefaultCommand(new Spitter2022StopCMD(spitter));
    }
  }

  private void initShooter2022() {
    if (RobotConstants.get().hasLlamaNeck2022()
        && RobotConstants.get().hasIndexer2022()
        && RobotConstants.get().hasSpitter2022()) {
      shooter2022 = new Shooter2022(indexer, llamaNeck, spitter);
        }
  }

  private void configureShooter2022() {
    if (RobotConstants.get().hasLlamaNeck2022()
        && RobotConstants.get().hasIndexer2022()
        && RobotConstants.get().hasSpitter2022()) {
      if (operatorShooterFlywheel.get()) {
        shooter2022.setDefaultCommand(
            new Shooter2022IdleCMD(shooter2022));
      } else {
        shooter2022.setDefaultCommand(
            new Shooter2022StopCMD(shooter2022));
      }

      operatorShooterFlywheel
          .whenPressed(
              new Shooter2022SetDefaultCMD(
                  shooter2022, new Shooter2022IdleCMD(shooter2022)))
          .whenReleased(
              new Shooter2022SetDefaultCMD(
                  shooter2022, new Shooter2022StopCMD(shooter2022)));
      operatorShooterShoot.whenPressed(
          new Shooter2022ShootTargetCMD(shooter2022));
      operatorIntakeRollerBackward.whenHeld(
          new Shooter2022FlushBallCMD(shooter2022));
    }
  }

  private void initHubCameraLED() {
    if (RobotConstants.get().hasHubCameraLED()) {
      hubCameraLED = new HubCameraLED();
      hubCameraLED.setDefaultCommand(new HubCameraLEDEnable(hubCameraLED));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(new Shooter2022IdleTargetCMD(shooter2022),
        // new ParallelRaceGroup(new Shooter2022IdleCMD(shooter2022),
            new SequentialCommandGroup(
                new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(),
                    new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)), false)//,
                // new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(),
                //     new Pose2d(-2, 0, Rotation2d.fromDegrees(0)), true)
                    )
                ),
        // new Shooter2022ShootTargetCMD(shooter2022));
        new Shooter2022ShootSpeedCMD(shooter2022, () -> Spitter2022.getFlywheelVelocity(Units.feetToMeters(9))));
    // return new ParallelRaceGroup(new Shooter2022IdleCMD(shooter2022), new
    // SequentialCommandGroup(new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0,
    // 0, new Rotation2d()), List.of(), new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
    // false), new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new
    // Rotation2d()), List.of(), new Pose2d(-2, 0, Rotation2d.fromDegrees(0)),
    // true)));
  }
}
