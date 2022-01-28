package frc.robot.commands;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Trigger2022;

public class Shooter2022FlushCMD extends CommandBase {

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Shooter2022FlushCMD.class.getName());

    private final Trigger2022 trigger;
    private final LlamaNeck2022 llamaNeck;
    private final Spitter2022 spitter;

    private final Command llamaNeckBackward;

    private final Command triggerBackward;

    private final Command spitterStop;

    public Shooter2022FlushCMD(Trigger2022 trigger, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.trigger = trigger;
        this.llamaNeck = llamaNeck;
        this.spitter = spitter;

        this.llamaNeckBackward = new LlamaNeck2022BackwardCMD(llamaNeck);
        this.triggerBackward = new Trigger2022BackwardCMD(trigger);
        this.spitterStop = new Spitter2022StopCMD(spitter);
    }

    @Override
    public void initialize() {
        LOGGER.info("Flushing system");
        llamaNeckBackward.schedule();
        triggerBackward.schedule();
        spitterStop.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
