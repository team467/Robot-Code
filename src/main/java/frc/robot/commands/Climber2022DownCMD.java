package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;

import java.util.function.Supplier;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Climber2022DownCMD extends CommandBase {
    private final Climber2022 climber;
    private final Supplier<Boolean> fullDown;

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022DownCMD.class.getName());



    //@param????

    public Climber2022DownCMD(Climber2022 climber, Supplier<Boolean> fullDown) {
        this.climber = climber;
        this.fullDown = fullDown;

        addRequirements(climber);
    }


    @Override 
    public void initialize(){
        LOGGER.info("Climber going down"); //don't know what to make the logs
    }


    public void execute() {
        if (fullDown.get()) {
            climber.downFull();
        } else {
            climber.downSafe();
        }
    }


    @Override 
    public void end(boolean interrupted) {}


    @Override 
    public boolean isFinished(){
        return false;
    }

}

