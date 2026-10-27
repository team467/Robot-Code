package frc.robot.subsystems.indexer;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Bindings;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;

public class IndexerSpark implements Indexer {

  SparkFlywheel flywheel;

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
    State reversing = machine.addState(
        flywheel.run(coro -> {
          while (true) {
            flywheel.reverse();
            coro.yield();
          }
        }).named("Indexer Reverse")
    );

    idle.switchTo(indexing).when(() -> Bindings.index);
    idle.switchTo(reversing).when(() -> Bindings.outtake);

    indexing.switchTo(idle).when(() -> !Bindings.index && !Bindings.outtake);
    indexing.switchTo(reversing).when(() -> Bindings.outtake);

    reversing.switchTo(idle).when(() -> !Bindings.outtake && !Bindings.index);
    reversing.switchTo(indexing).when(() -> !Bindings.outtake && Bindings.index);

    machine.setInitialState(idle);

    return machine;
  }
}
