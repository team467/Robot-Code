// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake2020Roller;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake2020RollerInCMD extends CommandBase {
  private final Intake2020Roller intakeRoller;

  /**
   * Creates a new Intake2020GrabberInCMD.
   *
   * @param intakeArm The subsystem used by this command.
   */
  public Intake2020RollerInCMD(Intake2020Roller intakeRoller) {
    this.intakeRoller = intakeRoller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Intake2020GrabberInCMD");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRoller.rollerIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
