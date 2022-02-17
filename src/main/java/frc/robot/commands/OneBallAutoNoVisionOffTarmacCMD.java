package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous, shoots preloaded ball, and drives off tarmac. */
public class OneBallAutoNoVisionOffTarmacCMD extends SequentialCommandGroup {
  int TARMAC_DISTANCE = 0;

  public OneBallAutoNoVisionOffTarmacCMD(
      Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
    addCommands(
        new Shooter2022ShootCMD(shooter, indexer, llamaNeck, spitter),
        new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE));
  }
}
