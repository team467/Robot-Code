// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class MoveRelative extends CommandBase {
  /** Creates a new MoveRelative. */
  Drive drive;

  public MoveRelative(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // hypot + angle,  find base and hight , set angle to zero (rotate -angle found)
    double yaw = drive.getYaw() % 360;
    double tagAngle = 20;
    double distance = 32;

    double angle = yaw + tagAngle % 360;

    double x = Math.cos(angle) * distance;
    double y = Math.sqrt(Math.pow(distance, 2) - (Math.pow(x, 2)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
