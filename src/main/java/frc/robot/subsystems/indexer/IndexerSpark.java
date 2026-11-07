package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.FEEDUP_VOLT;
import static org.wpilib.units.Units.Amps;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Bindings;
import frc.robot.Schematic;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.epilogue.Logged;
import org.wpilib.hardware.bus.CANPort;

@Logged
public class IndexerSpark implements Indexer {

  SparkFlywheel flywheel = new SparkFlywheel(
      CANPort.CAN_D0,
      Schematic.indexerFeedupCanId,
      false,
      FEEDUP_VOLT,
      Amps.of(12));

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

    idle.switchTo(indexing).when(() -> Bindings.shooterIsShooting);
    idle.switchTo(reversing).when(() -> Bindings.outtake);

    indexing.switchTo(idle).when(() -> !Bindings.shooterIsShooting && !Bindings.outtake);
    indexing.switchTo(reversing).when(() -> Bindings.outtake);

    reversing.switchTo(idle).when(() -> !Bindings.outtake && !Bindings.shooterIsShooting);
    reversing.switchTo(indexing).when(() -> !Bindings.outtake && Bindings.shooterIsShooting);

    machine.setInitialState(idle);

    return machine;
  }
}
