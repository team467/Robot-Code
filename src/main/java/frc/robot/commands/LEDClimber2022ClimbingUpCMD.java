package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDClimber2022;

public class LEDClimber2022ClimbingUpCMD extends CommandBase{
    

    private LEDClimber2022 ledClimber;
    private int color = 0;

    public LEDClimber2022ClimbingUpCMD(LEDClimber2022 ledClimber) {
        this.ledClimber = ledClimber;

        addRequirements(ledClimber);
    }

    @Override
    public void execute() {
       
    }
}
