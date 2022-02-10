// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter2022;

public class Shooter2022SetDefaultCMD extends InstantCommand {
  private final Shooter2022 shooter;
  private final Command defaultCommand;

  /** Creates a new Shooter2022SetDefaultCMD. */
  public Shooter2022SetDefaultCMD(Shooter2022 shooter, Command defaultCommand) {
    this.shooter = shooter;
    this.defaultCommand = defaultCommand;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setDefaultCommand(defaultCommand);
  }

}
