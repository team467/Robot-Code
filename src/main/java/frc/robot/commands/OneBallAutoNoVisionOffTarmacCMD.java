package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/** Autonomous, shoots preloaded ball, and drives off tarmac. */
public class OneBallAutoNoVisionOffTarmacCMD extends SequentialCommandGroup {
  int TARMAC_DISTANCE = 0;

  public OneBallAutoNoVisionOffTarmacCMD(
      Shooter2022 shooter, Drivetrain drivetrain, Gyro gyro) {
    addCommands(
        new Shooter2022ShootCMD(shooter),
        new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE));
  }
}
