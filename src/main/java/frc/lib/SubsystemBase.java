package frc.lib;

import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.StagedCommandBuilder;
import java.util.function.Consumer;

public abstract class SubsystemBase implements Mechanism {
  public NeedsNameBuilderStage run(Consumer<Coroutine> commandBody) {
    return new StagedCommandBuilder().requiring(this).executing(
        commandBody.andThen(_ -> periodic())
    );
  }

   public abstract void periodic();
}
