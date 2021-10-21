// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDriveCMD;
import frc.robot.commands.ClimberDownCMD;
import frc.robot.commands.ClimberStopCMD;
import frc.robot.commands.ClimberEnableCMD;
import frc.robot.commands.ClimberUpCMD;
import frc.robot.controllers.XboxController467.Buttons;
import frc.robot.controllers.CustomController2020;
import frc.robot.controllers.XboxController467.Axes;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private Climber climber = null;

  // User interface objects
  private final Joystick controller = new Joystick(0);
  private final Joystick op = new Joystick(1);
  private final JoystickButton buttonA = new JoystickButton(controller, Buttons.A.value);
  private final JoystickButton buttonB = new JoystickButton(controller, Buttons.B.value);
  private final JoystickButton buttonX = new JoystickButton(controller, Buttons.X.value);
  private final JoystickButton buttonY = new JoystickButton(controller, Buttons.Y.value);
  private final JoystickButton buttonBack = new JoystickButton(controller, Buttons.Back.value);
  private final JoystickButton buttonStart = new JoystickButton(controller, Buttons.Start.value);
  private final JoystickButton povUp = new JoystickButton(controller, Buttons.POVup.value);
  private final JoystickButton povDown = new JoystickButton(controller, Buttons.POVdown.value);
  private final JoystickButton povLeft = new JoystickButton(controller, Buttons.POVleft.value);
  private final JoystickButton povRight = new JoystickButton(controller, Buttons.POVright.value);
  private final JoystickButton leftBumper = new JoystickButton(controller, Buttons.BumperLeft.value);
  private final JoystickButton rightBumper = new JoystickButton(controller, Buttons.BumperRight.value);
  private final JoystickButton climbLock = new JoystickButton(op, CustomController2020.Buttons.CLIMB_LOCK.value);
  private final JoystickButton climbUp = new JoystickButton(op, CustomController2020.Buttons.CLIMB_UP.value);
  private final JoystickButton climbDown = new JoystickButton(op, CustomController2020.Buttons.CLIMB_DOWN.value);

  public RobotContainer() {
    // The default command is run when no other commands are active for the subsystem.
    drivetrain.setDefaultCommand(new ArcadeDriveCMD(drivetrain,
        () -> -controller.getRawAxis(Axes.LeftY.value),
        () ->  controller.getRawAxis(Axes.RightX.value)
    ));

    if (Constants.HAS_CLIMBER) {
      climber = new Climber();
      climber.setDefaultCommand(new ClimberStopCMD(climber));
    }

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
    if (Constants.HAS_CLIMBER) {
      climbLock.whenPressed(new ClimberEnableCMD(climber));
      climbUp.whenHeld(new ClimberUpCMD(climber));
      climbDown.whenHeld(new ClimberDownCMD(climber));
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
