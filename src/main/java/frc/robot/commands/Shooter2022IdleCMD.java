package frc.robot.commands;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Trigger2022;

public class Shooter2022IdleCMD extends CommandBase {

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Shooter2022IdleCMD.class.getName());

    private final Trigger2022 trigger;
    private final LlamaNeck2022 llamaNeck;
    private final Spitter2022 spitter;

    private final Command llamaNeckStop;
    private final Command llamaNeckIdle;

    private final Command triggerStop;
    private final Command triggerIdle;

    private final Command spitterStop;

    public Shooter2022IdleCMD(Trigger2022 trigger, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.trigger = trigger;
        this.llamaNeck = llamaNeck;
        this.spitter = spitter;

        this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
        this.llamaNeckIdle = new LlamaNeck2022IdleCMD(llamaNeck);

        this.triggerStop = new Trigger2022StopCMD(trigger);
        this.triggerIdle = new Trigger2022IdleCMD(trigger);

        this.spitterStop = new Spitter2022StopCMD(spitter);
    }

    @Override
    public void initialize() {
        LOGGER.info("Idling system...");
        triggerIdle.schedule();
        llamaNeckIdle.schedule();
        spitterStop.schedule();
    }

    @Override
    public void execute() {
        if (llamaNeck.getUpperLimitSwitch()) {
            LOGGER.info("Upper limit switch was activated. Stop trigger.");
            triggerStop.schedule();

            if (llamaNeck.getLowerLimitSwitch()) {
                LOGGER.info("Lower limit switch was activated. Stop llama neck.");
                llamaNeckStop.schedule();
            }
        } else {
            triggerIdle.schedule();
            llamaNeckIdle.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
