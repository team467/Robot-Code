package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Shooter2022;

/** Autonomous, shoots preloaded ball, and drives off tarmac. */
public class OneBallAutoNoVisionOffTarmacCMD extends SequentialCommandGroup {
  public OneBallAutoNoVisionOffTarmacCMD(
      Shooter2022 shooter, Drivetrain drivetrain, Gyro gyro) {
    addCommands(
        new GoToDistanceAngleCMD(drivetrain, gyro, 1.075, 0.0, true),
        new WaitCommand(0.5),
        new Shooter2022ShootTargetCMD(shooter, Units.feetToMeters(5.5)),
        new WaitCommand(0.5),
        new GoToDistanceAngleCMD(drivetrain, gyro, 1.075, 0.0, true)
    );

  }
}
