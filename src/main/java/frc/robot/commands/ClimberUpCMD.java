package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber2020;

public class ClimberUpCMD extends CommandBase {
    private final Climber2020 climber;

    public ClimberUpCMD(Climber2020 climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climber.isEnabled())
            climber.up();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
