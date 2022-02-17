package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous, shoots pre-loaded ball, and stays on tarmac. */
public class OneBallAutoNoVisionOnTarmacCMD extends SequentialCommandGroup {
  public OneBallAutoNoVisionOnTarmacCMD(
      Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
    addCommands(new Shooter2022ShootCMD(shooter, indexer, llamaNeck, spitter));
  }
}
