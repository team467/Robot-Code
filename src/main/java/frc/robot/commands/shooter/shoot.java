// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class shoot extends Command {
  private boolean readyToShoot;
  private final Shooter shooter;
  /** Creates a new shoot. */
  public shoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToShoot = shooter.getHoldingNote() && shooter.getFlywheelSpeedIsReady();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (readyToShoot) {
      shooter.setIndexerVoltage(ShooterConstants.indexerFowardVoltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setIndexerVoltage(ShooterConstants.indexerHoldVoltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.getHoldingNote()) {
      return true;
    }
    else {
      return false;
    }
  }
}
