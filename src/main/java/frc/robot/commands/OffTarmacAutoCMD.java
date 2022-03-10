package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

/** Autonomous Move backwards off the tarmac until bumpers clear the area (2 points). */
public class OffTarmacAutoCMD extends SequentialCommandGroup {

  public OffTarmacAutoCMD(Drivetrain drivetrain, Gyro gyro) {
    addCommands(new GoToDistanceAngleCMD(drivetrain, gyro, 2.15, 0.0, true));
  }
}
