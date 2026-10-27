package frc.robot.subsystems.magicCarpet;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Bindings;
import frc.robot.Schematic;
import org.wpilib.command3.StateMachine;
import org.wpilib.hardware.bus.CANPort;

public class MagicCarpetSpark implements MagicCarpet {

  SparkFlywheel flywheel;

  public MagicCarpetSpark() {
    flywheel = new SparkFlywheel(CANPort.CAN_D0, Schematic.magicCarpetCanId,
        MagicCarpetConstants.MOTOR_INVERTED, MagicCarpetConstants.BELT_VOLTS,
        MagicCarpetConstants.CURRENT_LIMIT);
  }

  @Override
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Magic Carpet");

    var idle = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.stop();
        coro.yield();
      }
    }).named("Magic Carpet Stop"));
    var run = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.run();
        coro.yield();
      }
    }).named("Magic Carpet Run"));
    var reverse = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.run();
        coro.yield();
      }
    }).named("Magic Carpet Reverse"));

    idle.switchTo(run).when(() -> Bindings.intake || Bindings.index);
    idle.switchTo(reverse).when(() -> Bindings.outtake);

    run.switchTo(idle).when(() -> !Bindings.intake && !Bindings.index);
    run.switchTo(reverse).when(() -> Bindings.outtake);

    reverse.switchTo(idle).when(() -> !Bindings.intake && !Bindings.index && !Bindings.outtake);
    reverse.switchTo(run).when(() -> !Bindings.outtake && (Bindings.intake || Bindings.index));

    machine.setInitialState(idle);

    return machine;
  }
}
