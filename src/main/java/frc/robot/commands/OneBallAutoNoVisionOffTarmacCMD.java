// Autonomous, shoots preloaded ball, and drives off tarmac.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class OneBallAutoNoVisionOffTarmacCMD extends SequentialCommandGroup {
    public OneBallAutoNoVisionOffTarmacCMD(Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter)
    {        
        int TARMAC_DISTANCE = 0;
        super
        (
            new Shooter2022ShootCMD(shooter, indexer, llamaNeck, spitter),
            new DriveDistanceCMD(drivetrain, gyro, TARMAC_DISTANCE)
        );

    }

}
