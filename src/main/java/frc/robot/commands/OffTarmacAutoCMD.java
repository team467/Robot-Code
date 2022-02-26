package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/** Autonomous Move backwards off the tarmac until bumpers clear the area (2 points). */
public class OffTarmacAutoCMD extends SequentialCommandGroup {
  int TARMAC_DISTANCE = 0;

  public OffTarmacAutoCMD(Drivetrain drivetrain, Gyro gyro) {

    addCommands(new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE));
<<<<<<< Updated upstream
=======

    // DriveDistanceAngle(0, );
>>>>>>> Stashed changes
  }
}
