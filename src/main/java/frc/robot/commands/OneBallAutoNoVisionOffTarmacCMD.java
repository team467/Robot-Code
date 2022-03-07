package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Shooter2022;

/** Autonomous, shoots preloaded ball, and drives off tarmac. */
public class OneBallAutoNoVisionOffTarmacCMD extends SequentialCommandGroup {
  public OneBallAutoNoVisionOffTarmacCMD(
      Shooter2022 shooter, Drivetrain drivetrain, Gyro gyro) {
    addCommands(
        new Shooter2022ShootTargetCMD(shooter, Units.feetToMeters(3)),
        new GoToDistanceAngleCMD(drivetrain, gyro, 2.15, 0.0, true)
    );

  }
}
