package frc.robot.commands;

import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climber2022ZeroCMD extends CommandBase {
    private final Climber2022 climber;

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022EnableCMD.class.getName());

    public Climber2022ZeroCMD(Climber2022 climber) {
        this.climber = climber;

        addRequirements(climber);
    }


    @Override 
    public void initialize() {
        climber.setLeftSpeed(-RobotConstants.get().climber2022ZeroingSpeed());
        climber.setRightSpeed(-RobotConstants.get().climber2022ZeroingSpeed());
        LOGGER.debug("Climber enabled");
    }

    @Override 
    public void execute() {
        if (climber.getLeftLimitSwitch()) {
            climber.stopLeft();
            climber.resetLeftPosition();
        }

        if (climber.getRightLimitSwitch()) {
            climber.stopRight();
            climber.resetRightPosition();
        }
    }


    @Override 
    public void end(boolean interrupted) {
        climber.stop();
        climber.resetLeftPosition();
        climber.resetRightPosition();
    }


    @Override 
    public boolean isFinished() {
        if (!RobotConstants.get().climber2022HasLimitSwitch()) return true;
        return climber.getLeftLimitSwitch() && climber.getRightLimitSwitch();
    }
}