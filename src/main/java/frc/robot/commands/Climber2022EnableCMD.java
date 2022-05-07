package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Climber2022EnableCMD extends CommandBase {
    private final Climber2022 climber;

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022EnableCMD.class.getName());




    public Climber2022EnableCMD(Climber2022 climber) {
        this.climber = climber;

        addRequirements(climber);
    }


    @Override 
    public void initialize() {
        climber.enable();
        LOGGER.debug("Climber enabled");
    }

    @Override 
    public void execute(){
    }


    @Override 
    public void end(boolean interrupted) {}


    @Override 
    public boolean isFinished() {
        return true;
    }
}
