package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter2022;

/** Autonomous, shoots pre-loaded ball, and stays on tarmac. */
public class OneBallAutoNoVisionOnTarmacCMD extends SequentialCommandGroup {
  public OneBallAutoNoVisionOnTarmacCMD(
      Shooter2022 shooter) {
    addCommands(new Shooter2022ShootCMD(shooter));
  }
}
