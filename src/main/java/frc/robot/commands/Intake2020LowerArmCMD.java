// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake2020Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Intake2020LowerArmCMD extends CommandBase {
  private final Intake2020Arm intakeArm;

  /**
   * Creates a new Intake2020LowerArmCMD.
   *
   * @param intakeArm The subsystem used by this command.
   */
  public Intake2020LowerArmCMD(Intake2020Arm intakeArm) {
    this.intakeArm = intakeArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Intake2020LowerArmCMD");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeArm.lowerArm();
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
