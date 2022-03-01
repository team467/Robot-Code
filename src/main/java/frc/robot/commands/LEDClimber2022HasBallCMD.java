package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDClimber2022;

public class LEDClimber2022HasBallCMD extends CommandBase {

    private LEDClimber2022 ledClimber;
    private Color teamColor = Color.kBlue; 

    public LEDClimber2022HasBallCMD(LEDClimber2022 ledClimber) {
        this.ledClimber = ledClimber;

            addRequirements(ledClimber);
    }

        @Override
        public void initialize() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            teamColor = Color.kRed;
        } else {
            teamColor = Color.kBlue;
        }
    }

        @Override
        public void execute() {
           for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
                ledClimber.setRGB(i,255,192,203); //how to get half of led to be lit 
           }

        ledClimber.sendData();   
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
