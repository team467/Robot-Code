package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
    public boolean hasBall() {
        return inputs.ballAtSwitch;
    }

    public Command stop() {
        return Commands.runOnce(() ->{io.stop();}, this);
    }

    public Command indexUntilSwitch() {
        return Commands.run(() -> io.setPercent(IndexerConstants.INDEX_PERCENT), this)
                .until(io::isSwitchPressed)
                .finallyDo(interrupted -> io.stop());
    }

    public Command indexIntoShooter() {
        return Commands.run(() -> io.setPercent(IndexerConstants.INDEX_PERCENT), this)
                .until(() -> !inputs.ballAtSwitch)
                .finallyDo(interrupted -> io.stop());
    }

    public Command reverse() {
        return Commands.startEnd(
                () -> io.setPercent(IndexerConstants.REVERSE_INDEX_PERCENT),
                () -> io.setPercent(0),
                this
        );
    }

}