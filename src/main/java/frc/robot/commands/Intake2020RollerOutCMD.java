// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake2020Roller;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake2020RollerOutCMD extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake2020Roller intakeRoller;

  /**
   * Creates a new Intake2020GrabberOutCMD.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intake2020RollerOutCMD(Intake2020Roller subsystem) {
    this.intakeRoller = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Intake2020GrabberOutCMD");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRoller.rollerOut();
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
