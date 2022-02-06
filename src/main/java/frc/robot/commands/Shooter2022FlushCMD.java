package frc.robot.commands;

import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Indexer2022;

public class Shooter2022FlushCMD extends CommandBase {

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Shooter2022FlushCMD.class.getName());

    private final Command llamaNeckBackward;

    private final Command indexerBackward;

    private final Command spitterStop;

    public Shooter2022FlushCMD(Shooter2022 shooter, Indexer2022 indexer, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.llamaNeckBackward = new LlamaNeck2022BackwardCMD(llamaNeck);
        this.indexerBackward = new Indexer2022BackwardCMD(indexer);
        this.spitterStop = new Spitter2022StopCMD(spitter);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        LOGGER.debug("Flushing system");
        llamaNeckBackward.schedule();
        indexerBackward.schedule();
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
