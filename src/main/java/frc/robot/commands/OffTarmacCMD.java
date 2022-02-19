package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/** Autonomous Move backwards off the tarmac until bumpers clear the area (2 points). */
public class OffTarmacCMD extends SequentialCommandGroup {
  int TARMAC_DISTANCE = 0;

  public OffTarmacCMD(Drivetrain drivetrain, Gyro gyro) {

    addCommands(new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE));
  }
}
