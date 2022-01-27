package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Trigger2022;

public class Robot2022PanicCMD extends CommandBase {

    private final Trigger2022 trigger;
    private final LlamaNeck2022 llamaNeck;
    private final Spitter2022 spitter;
    private final Drivetrain drivetrain;

    public Robot2022PanicCMD(Trigger2022 trigger, LlamaNeck2022 llamaNeck, Spitter2022 spitter, Drivetrain drivetrain) {
        super();

        this.trigger = trigger;
        this.llamaNeck = llamaNeck;
        this.spitter = spitter;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        trigger.getCurrentCommand().cancel();
        llamaNeck.getCurrentCommand().cancel();
        spitter.getCurrentCommand().cancel();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
