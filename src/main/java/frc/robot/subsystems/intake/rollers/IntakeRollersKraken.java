package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.mechanisms.flywheels.KrakenFlywheel;
import frc.robot.Bindings;
import frc.robot.Schematic;
import org.wpilib.command3.StateMachine;
import org.wpilib.hardware.bus.CANPort;

public class IntakeRollersKraken implements IntakeRollers {

  KrakenFlywheel flywheel;

  public IntakeRollersKraken() {
    flywheel = new KrakenFlywheel("Intake Rollers", CANPort.CAN_D0, Schematic.intakeMotorCanId,
        InvertedValue.CounterClockwise_Positive, INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT, INTAKE_VOLTS);
  }

  @Override
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Intake Rollers");

    var idle = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.stop();
        coro.yield();
      }
    }).named("Intake Rollers Idle"));
    var run = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.run();
        coro.yield();
      }
    }).named("Intake Rollers Running"));
    var reverse = machine.addState(flywheel.run(coro -> {
      while (true) {
        flywheel.reverse();
        coro.yield();
      }
    }).named("Intake Rollers Reversing"));

    idle.switchTo(run).when(() -> Bindings.intake);
    idle.switchTo(reverse).when(() -> Bindings.outtake);

    run.switchTo(idle).when(() -> !Bindings.intake);
    run.switchTo(reverse).when(() -> Bindings.outtake);

    reverse.switchTo(idle).when(() -> !Bindings.intake && !Bindings.outtake);
    reverse.switchTo(run).when(() -> !Bindings.outtake && Bindings.intake);

    machine.setInitialState(idle);

    return machine;
  }
}
