// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake2020Arm;
import frc.robot.subsystems.Intake2020Roller;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Intake2020RaiseArmCMD extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake2020Arm m_subsystemArm;
  private final Intake2020Roller m_subsystemRoller;

  /**
   * Creates a new Intake2020RaiseArmCMD.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intake2020RaiseArmCMD(Intake2020Arm armsubsystem, Intake2020Roller rollersubsystem) {
    m_subsystemArm = armsubsystem;
    m_subsystemRoller = rollersubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armsubsystem);
    addRequirements(rollersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Intake2020RaiseArmCMD");
    m_subsystemRoller.rollerStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystemArm.raiseArm();
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
