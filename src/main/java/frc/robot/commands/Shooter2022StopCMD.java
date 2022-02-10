// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;

public class Shooter2022StopCMD extends CommandBase {
  private final LlamaNeck2022 llamaNeck;
  private final Spitter2022 spitter;
  private final Indexer2022 indexer;

  private final Command llamaNeckStop;
  private final Command spitterStop;
  private final Command indexerStop;

  /** Creates a new Shooter2022Stop. */
  public Shooter2022StopCMD(Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
    super();

    this.llamaNeck = llamaNeck;
    this.spitter = spitter;
    this.indexer = indexer;

    this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
    this.spitterStop = new Spitter2022StopCMD(spitter);
    this.indexerStop = new Indexer2022StopCMD(indexer);

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    llamaNeckStop.schedule();
    spitterStop.schedule();
    indexerStop.schedule();
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
