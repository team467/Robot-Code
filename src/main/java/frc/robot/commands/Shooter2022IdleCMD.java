package frc.robot.commands;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Indexer2022;

public class Shooter2022IdleCMD extends CommandBase {

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Shooter2022IdleCMD.class.getName());

    private final LlamaNeck2022 llamaNeck;

    private final Command llamaNeckStop;
    private final Command llamaNeckIdle;

    private final Command indexerStop;
    private final Command indexerIdle;

    private final Command spitterStop;

    public Shooter2022IdleCMD(Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.llamaNeck = llamaNeck;

        this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
        this.llamaNeckIdle = new LlamaNeck2022IdleCMD(llamaNeck);

        this.indexerStop = new Indexer2022StopCMD(indexer);
        this.indexerIdle = new Indexer2022IdleCMD(indexer);

        this.spitterStop = new Spitter2022StopCMD(spitter);
    }

    @Override
    public void initialize() {
        LOGGER.debug("Idling system...");
        indexerIdle.schedule();
        llamaNeckIdle.schedule();
        spitterStop.schedule();
    }

    @Override
    public void execute() {
        if (llamaNeck.getUpperLimitSwitch()) {
            LOGGER.debug("Upper limit switch was activated. Stop indexer.");
            indexerStop.schedule();

            if (llamaNeck.getLowerLimitSwitch()) {
                LOGGER.debug("Lower limit switch was activated. Stop llama neck.");
                llamaNeckStop.schedule();
            }
        } else {
            indexerIdle.schedule();
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
