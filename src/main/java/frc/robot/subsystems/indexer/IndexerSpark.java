package frc.robot.subsystems.indexer;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Bindings;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;

public class IndexerSpark implements Indexer {

  SparkFlywheel flywheel;

  @Override
  public void feed() {
    flywheel.run();
  }

  @Override
  public void reverse() {
    flywheel.reverse();
  }

  @Override
  public void stop() {
    flywheel.stop();
  }

  @Override
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Indexer");

    State idle = machine.addState(
        flywheel.run(coro -> {
          while (true) {
            flywheel.stop();
            coro.yield();
          }
        }).named("Indexer Stop")
    );
    State indexing = machine.addState(
        flywheel.run(coro -> {
          while (true) {
            flywheel.run();
            coro.yield();
          }
        }).named("Indexer Feed")
    );

    idle.switchTo(indexing).when(() -> Bindings.index);
    indexing.switchTo(idle).when(() -> !Bindings.index);

    machine.setInitialState(idle);

    return machine;
  }
}
