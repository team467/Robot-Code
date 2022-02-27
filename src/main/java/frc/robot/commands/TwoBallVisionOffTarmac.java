package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Drivetrain;

/** Autonomous mode:
 * 1. The robot shoots the pre-loaded ball.
 * 2. It drives to get the second ball.
 * 3. It then drives up and shoots it.
 */

public class TwoBallVisionOffTarmac extends SequentialCommandGroup{
    // Values will change due to vision.
    int GET_BALL_DISTANCE = 0;

    public TwoBallVisionOffTarmac(
        Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter, Drivetrain drivetrain) {
      addCommands(
          // These will also change due to vision.
          new Shooter2022ShootCMD(shooter),
          //new DriveDistanceCMD(drivetrain, gyro, GET_BALL_DISTANCE),
          new ArcadeDriveCMD (drivetrain, () -> 0.2, () -> 3.0));
          new Shooter2022ShootCMD (shooter);
          //new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE),
          new ArcadeDriveCMD (drivetrain, () -> 0.2, () -> 3.0);

    }
    
}
