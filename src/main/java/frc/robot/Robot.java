// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controllers.ControllerQueue;
import frc.robot.tuning.TunerManager;

import java.io.IOException;

import frc.robot.logging.RobotLogManager;
import org.apache.logging.log4j.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final Logger LOGGER = RobotLogManager.getMainLogger(Robot.class.getName());

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CommandScheduler commandScheduler;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    m_robotContainer = new RobotContainer();
    commandScheduler = CommandScheduler.getInstance();

    // Mounting USB
    ProcessBuilder builder = new ProcessBuilder();
    builder.command("sudo", "mount", "/dev/sda1", "/media");
    try {
      builder.start();
    } catch (IOException e) {
      e.printStackTrace();
    }  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    ControllerQueue.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.configureButtonBindings();
    m_robotContainer.getDisabledCommand().getDefaultCommand().initialize();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.getDisabledCommand().getDefaultCommand().execute();
  }
  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.disableDemoMode();
    CommandScheduler.getInstance().clearButtons();
    m_robotContainer.clearDefaultCommands();
    // CommandScheduler.getInstance().cancelAll();
    // CommandScheduler.getInstance().enable();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.andThen(m_robotContainer::configureButtonBindings).schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.disableDemoMode();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.configureButtonBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().clearButtons();
    CommandScheduler.getInstance().cancelAll();

    // You need to enable the command scheduler in test mode
    CommandScheduler.getInstance().enable();
    TunerManager.getTunerManager().getTunerChoice().initializeTuner();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
