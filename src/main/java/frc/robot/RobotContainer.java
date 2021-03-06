// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDriveCMD;
import frc.robot.commands.BlankDefaultCMD;
import frc.robot.commands.Indexer2022StopCMD;
import frc.robot.commands.Climber2022DisableCMD;
import frc.robot.commands.Climber2022EnableCMD;
import frc.robot.commands.Climber2022StopCMD;
import frc.robot.commands.Climber2022UpCMD;
import frc.robot.commands.Climber2022DownCMD;
import frc.robot.commands.ClimberDownCMD;
import frc.robot.commands.ClimberEnableCMD;
import frc.robot.commands.ClimberStopCMD;
import frc.robot.commands.ClimberUpCMD;
import frc.robot.commands.DrivetrainNoneCMD;
import frc.robot.commands.GoToDistanceAngleCMD;
import frc.robot.commands.GoToTargetCMD;
import frc.robot.commands.GoToTrajectoryCMD;
import frc.robot.commands.HubCameraLEDEnable;
import frc.robot.commands.Led2022UpdateCMD;
import frc.robot.commands.GoToTrajectoryCMD;
import frc.robot.commands.HubCameraLEDSmart;
import frc.robot.commands.LlamaNeck2022StopCMD;
import frc.robot.commands.OffTarmacAutoCMD;
import frc.robot.commands.OneBallAutoNoVisionOffTarmacCMD;
import frc.robot.commands.OneBallAutoNoVisionOnTarmacCMD;
import frc.robot.commands.Shooter2022FlushBallCMD;
import frc.robot.commands.Shooter2022IdleCMD;
import frc.robot.commands.Shooter2022IdleSpinupCMD;
import frc.robot.commands.Shooter2022IdleTargetCMD;
import frc.robot.commands.Shooter2022SetDefaultCMD;
import frc.robot.commands.Shooter2022ShootSpeedCMD;
import frc.robot.commands.Shooter2022ShootTargetCMD;
import frc.robot.commands.Shooter2022StopCMD;
import frc.robot.commands.ShooterRunFlywheelCMD;
import frc.robot.commands.ShooterStopFlywheelCMD;
import frc.robot.commands.ShooterTriggerForwardCMD;
import frc.robot.commands.ShooterTriggerStopCMD;
import frc.robot.controllers.ControllerQueue;
import frc.robot.commands.Spitter2022StopCMD;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.controllers.CustomController2020;
import frc.robot.controllers.CustomController2022;
import frc.robot.controllers.CustomControllerBase;
import frc.robot.controllers.XboxController467;
import frc.robot.led.LEDManager;
import frc.robot.subsystems.Climber2020;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Led2022;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubCameraLED;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2020;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Climber2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.tuners.DemoModeTuner;
import frc.robot.tuning.TunerManager;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

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
  private SendableChooser<Command> autoModes = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain = null;
  private Gyro gyro = null;
  private Shooter2020 shooter = null;
  private Led2022 led2022 = null;
  private LlamaNeck2022 llamaNeck = null;
  private Indexer2022 indexer = null;
  private Spitter2022 spitter = null;
  private Shooter2022 shooter2022 = null;
  private HubCameraLED hubCameraLED = null;
  private Climber2020 climber2020 = null;
  private Climber2022 climber2022 = null;

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
  private final CustomController2022 operatorJoystick = new CustomController2022(1);
  private final JoystickButton operatorFlush = operatorJoystick.getButton(CustomController2022.Buttons.FLUSH);
  private final JoystickButton operatorClimberLimits = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LIMITS);
  private final JoystickButton operatorShooterAuto = operatorJoystick.getButton(CustomController2022.Buttons.SHOOTER_AUTO);
  private final JoystickButton operatorEverything = operatorJoystick.getButton(CustomController2022.Buttons.EVERYTHING);
  private final JoystickButton operatorShoot = operatorJoystick.getButton(CustomController2022.Buttons.SHOOT);
  private final JoystickButton operatorClimberLock = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_LOCK);
  private final JoystickButton operatorClimberUp = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_UP);
  private final JoystickButton operatorClimberDown = operatorJoystick.getButton(CustomController2022.Buttons.CLIMBER_DOWN);


  public RobotContainer() {
    getTrajectories();

    initializeSubsystems();

    LEDManager.getInstance().init();

    initializeAutoCommands();

    TunerManager.getTunerManager().setDefaultTuner(new DemoModeTuner(this));

    // Configure the button bindings
    configureButtonBindings();
    ControllerQueue.getInstance().addController(operatorJoystick);
  }

  public void getTrajectories() {
    File path = Paths.get(Filesystem.getDeployDirectory().toString(), "paths",
        RobotConstants.get().name().toLowerCase().replace(" ", ""), "output").toFile();

    if (path.exists()) {
      for (File file : path.listFiles()) {
        try {
          trajectories.put(file.getName().replace(".wpilib.json", ""),
              TrajectoryUtil.fromPathweaverJson(file.toPath()));
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
  }

  public Led2022 getDisabledCommand() {
    return led2022;
  }

  public void initializeAutoCommands() {
    autoModes.setDefaultOption("Do nothing", new RunCommand(() -> {
    }));

    if (drivetrain != null && gyro != null) {
      autoModes.addOption("Off tarmac", new OffTarmacAutoCMD(drivetrain, gyro));
    }

    if (shooter2022 != null) {
      autoModes.addOption("One ball on tarmac", new OneBallAutoNoVisionOnTarmacCMD(shooter2022));
    }

    if (drivetrain != null && gyro != null && shooter2022 != null) {
      autoModes.addOption("One ball off tarmac", new OneBallAutoNoVisionOffTarmacCMD(shooter2022, drivetrain, gyro));

      autoModes.addOption("Two ball auto", new SequentialCommandGroup(
          new ParallelRaceGroup(
              new Shooter2022IdleTargetCMD(shooter2022),
              new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(),
                  new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)), false)),
          new Shooter2022ShootTargetCMD(shooter2022, Units.feetToMeters(9))));
    }

    Shuffleboard.getTab("Auto").add("Autonomous Selector", autoModes).withPosition(3, 2).withSize(4, 1);
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
    initShooter2020();
    initIndexer2022();
    initClimber2022();
    initSpitter2022();
    initLlamaNeck2022();
    initShooter2022();
    initHubCameraLED();
    initLed2022();
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
    configureClimber2022();
    configureShooter2020();
    configureLlamaNeck2022();
    configureIndexer2022();
    configureSpitter2022();
    configureShooter2022();
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
      driverButtonY.whenPressed(new TurnToTargetCMD(drivetrain, gyro));
      // driverButtonY.whileHeld(new GoToTargetCMD(drivetrain, gyro));
      // driverButtonA.whenPressed(new TurnAngleCMD(drivetrain, gyro, 90));
      // driverButtonY.whileHeld(new GoToTrajectoryCMD(drivetrain, gyro,
      // trajectories.get("Reverse")));
      // driverButtonA.whileHeld(new GoToDistanceAngleCMD(drivetrain, gyro, 2.0, 0.0,
      // true));
      // driverButtonA.whenPressed(new GoToTrajectoryCMD(drivetrain, gyro, new
      // Pose2d(0, 0, new Rotation2d()), List.of(),
      // new Pose2d(-2, 0, Rotation2d.fromDegrees(0)), true));
      // driverButtonB.whenPressed(new GoToTrajectoryCMD(drivetrain, gyro, new
      // Pose2d(0, 0, new Rotation2d()), List.of(),
      // new Pose2d(2, 0, Rotation2d.fromDegrees(0)), false));
    }
  }

  private void initClimber2020() {
    if (RobotConstants.get().hasClimber2020()) {
      climber2020 = new Climber2020();
    }
  }

  private void configureClimber2020() {
    if (RobotConstants.get().hasClimber2020()) {
      climber2022.setDefaultCommand(new ClimberStopCMD(climber2020));
      operatorClimberLock.whenPressed(new ClimberEnableCMD(climber2020));
      operatorClimberUp.whenHeld(new ClimberUpCMD(climber2020));
      operatorClimberDown.whenHeld(new ClimberDownCMD(climber2020));
    }
  }

  private void initShooter2020() {
    if (RobotConstants.get().hasShooter2020()) {
      shooter = new Shooter2020();
    }
  }

  private void configureShooter2020() {
    if (RobotConstants.get().hasShooter2020()) {
      operatorEverything.whenPressed(new ShooterRunFlywheelCMD(shooter));
      operatorEverything.whenReleased(new ShooterStopFlywheelCMD(shooter));
      operatorShoot.whenPressed(new ShooterTriggerForwardCMD(shooter));
      operatorShoot.whenReleased(new ShooterTriggerStopCMD(shooter));
      operatorShoot.whenReleased(new ShooterTriggerStopCMD(shooter));
    }
  }

  private void initLed2022() {
    if (RobotConstants.get().hasLed2022()) {
      led2022 = new Led2022();
      if (RobotConstants.get().hasSpitter2022() 
        && RobotConstants.get().hasLlamaNeck2022()
        && RobotConstants.get().hasClimber2022()) {
          led2022.setDefaultCommand(new Led2022UpdateCMD(led2022, indexer, llamaNeck, spitter, shooter2022, climber2022));
        } else {
          led2022.setDefaultCommand(new Led2022UpdateCMD(led2022));
        }
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

  private void initClimber2022() {
    if (RobotConstants.get().hasClimber2022()) {
      climber2022 = new Climber2022();
      climber2022.setDefaultCommand(new Climber2022StopCMD(climber2022));
    }
  }

  private void configureClimber2022() {
    if (RobotConstants.get().hasClimber2022()) {
      operatorClimberLock.whenPressed(new Climber2022EnableCMD(climber2022));
      operatorClimberLock.whenReleased(new Climber2022DisableCMD(climber2022));
      operatorClimberUp.whileHeld(new Climber2022UpCMD(climber2022));
      operatorClimberDown.whileHeld(new Climber2022DownCMD(climber2022, operatorClimberLimits::get));
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
      if (operatorEverything.get()) {
        shooter2022.setDefaultCommand(
            new Shooter2022IdleSpinupCMD(shooter2022, () -> Spitter2022.getFlywheelVelocity(0.6)));
      } else {
        shooter2022.setDefaultCommand(
            new Shooter2022StopCMD(shooter2022));
      }

      operatorEverything
          .whenPressed(
              new Shooter2022SetDefaultCMD(
                  shooter2022, new Shooter2022IdleSpinupCMD(shooter2022, () -> Spitter2022.getFlywheelVelocity(0.6))))
          .whenReleased(
              new Shooter2022SetDefaultCMD(
                  shooter2022, new Shooter2022StopCMD(shooter2022)));
      operatorShoot.whenPressed(
          new Shooter2022ShootTargetCMD(shooter2022));
      operatorFlush.whenHeld(
          new Shooter2022FlushBallCMD(shooter2022));
    }
  }

  private void initHubCameraLED() {
    if (RobotConstants.get().hasHubCameraLED()) {
      hubCameraLED = new HubCameraLED();
      hubCameraLED.setDefaultCommand(new HubCameraLEDSmart(hubCameraLED, shooter2022));
    }
  }

  public void clearDefaultCommands() {
    if (drivetrain != null)
      drivetrain.setDefaultCommand(new BlankDefaultCMD(drivetrain));
    if (shooter2022 != null)
      shooter2022.setDefaultCommand(new BlankDefaultCMD(shooter2022));
    if (spitter != null)
      spitter.setDefaultCommand(new BlankDefaultCMD(spitter));
    if (llamaNeck != null)
      llamaNeck.setDefaultCommand(new BlankDefaultCMD(llamaNeck));
    if (indexer != null)
      indexer.setDefaultCommand(new BlankDefaultCMD(indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoModes.getSelected();
    // if (shooter2022 != null) {
    // return new SequentialCommandGroup(
    // new ParallelRaceGroup(
    // new Shooter2022IdleTargetCMD(shooter2022),
    // // new ParallelRaceGroup(new Shooter2022IdleCMD(shooter2022),
    // new SequentialCommandGroup(
    // new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()),
    // List.of(),
    // new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)), false)//,
    // // new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new
    // Rotation2d()), List.of(),
    // // new Pose2d(-2, 0, Rotation2d.fromDegrees(0)), true)
    // )
    // ),
    // new Shooter2022ShootTargetCMD(shooter2022,
    // Units.feetToMeters(9))).andThen(this::configureButtonBindings);
    // // new Shooter2022ShootSpeedCMD(shooter2022, () ->
    // Spitter2022.getFlywheelVelocity(Units.feetToMeters(9)))).andThen(() ->
    // configureButtonBindings(););
    // }
    // return new OneBallAutoNoVisionOnTarmacCMD(shooter2022).andThen(() ->
    // configureButtonBindings());
    // return new ParallelRaceGroup(new Shooter2022IdleCMD(shooter2022), new
    // SequentialCommandGroup(new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0,
    // 0, new Rotation2d()), List.of(), new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
    // false), new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new
    // Rotation2d()), List.of(), new Pose2d(-2, 0, Rotation2d.fromDegrees(0)),
    // true)));
  }

  public void enableDemoMode() {
    if (drivetrain != null) {
      drivetrain.setMaxSpeedMultiplier(RobotConstants.get().driveDemoModeMultiplier());
    }
  }

  public void disableDemoMode() {
    if (drivetrain != null) {
      drivetrain.setMaxSpeedMultiplier(RobotConstants.get().driveNormalModeMultiplier());
    }
  }
}
