// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDriveCMD;
import frc.robot.commands.ClimberDownCMD;
import frc.robot.commands.ClimberEnableCMD;
import frc.robot.commands.ClimberStopCMD;
import frc.robot.commands.ClimberUpCMD;
import frc.robot.commands.LEDTower2022ChaseBallCMD;
import frc.robot.commands.LEDTower2022ColorCycleCMD;
import frc.robot.commands.LEDTower2022FoundBallCMD;
import frc.robot.commands.LEDTower2022OffCMD;
import frc.robot.commands.LEDTower2022RainbowCMD;
import frc.robot.commands.ShooterRunFlywheelCMD;
import frc.robot.commands.ShooterStopFlywheelCMD;
import frc.robot.commands.ShooterTriggerForwardCMD;
import frc.robot.commands.ShooterTriggerStopCMD;
import frc.robot.controllers.CustomController2020;
import frc.robot.controllers.XboxController467;
import frc.robot.led.LEDManager;
import frc.robot.subsystems.Climber2020;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDTower2022;
import frc.robot.subsystems.Shooter2020;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain = null;
  private Climber2020 climber = null;
  private Shooter2020 shooter = null;
  private LEDTower2022 ledTower2022 = null;
  private LEDTower2022 ledTower20222 = null;

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
    initializeSubsystems();
    LEDManager.getInstance().init();
    // Configure the button bindings
    configureButtonBindings();
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
    initDrivetrain();
    initClimber2020();
    initShooter2020();
    initLEDTower2022();
  }

  public void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();
    configureDrivetrain();
    configureClimber2020();
    configureShooter2020();
    configureLEDTower2022();
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
              driverJoystick::getAdjustedTurnSpeed
      ));
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

  private void initLEDTower2022() {
    if (RobotConstants.get().hasLEDTower2022()) {
      ledTower2022 = new LEDTower2022();
      ledTower20222 = new LEDTower2022();
    }
  }

  private void configureLEDTower2022() {
    if (RobotConstants.get().hasLEDTower2022()) {
      ledTower2022.setDefaultCommand(new LEDTower2022ChaseBallCMD(ledTower2022));
      ledTower20222.setDefaultCommand(new LEDTower2022RainbowCMD(ledTower20222));
    }
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
