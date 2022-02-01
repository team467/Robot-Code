package frc.robot.commands;

import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberUp2022CMD extends CommandBase {
    private final Climber2022 climber;

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022.class.getName());


    public ClimberUp2022CMD(Climber2022 climber) {
        this.climber = climber;

        addRequirements(climber);
    }


    @Override
    public void initialize() {
        LOGGER.info("Climber going up");
    }


    @Override 
    public void execute() {
        climber.up();
    }

    @Override 
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished() {
        return false;
    }
}
