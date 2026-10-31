package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.mechanisms.flywheels.KrakenFlywheel;
import frc.lib.mechanisms.flywheels.ReversibleFlywheel;
import frc.robot.Bindings;
import frc.robot.Schematic;
import frc.robot.subsystems.intake.extend.IntakeExtend;
import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;
import org.wpilib.hardware.bus.CANPort;

public class Intake {

  private final ReversibleFlywheel rollers;
  private final IntakeExtend extend;

  public Intake(IntakeExtend extend) {
    this.rollers = new KrakenFlywheel("Intake Rollers", CANPort.CAN_D0, Schematic.intakeMotorCanId,
        InvertedValue.CounterClockwise_Positive, INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT, INTAKE_VOLTS);
    this.extend = extend;
  }

  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Intake Machine");

    var idling = machine.addState(extend.retract());
    var intaking = machine.addState(Command.noRequirements(coro -> {
      coro.fork(extend.extend()); // Bring out the intake
      while (true) {
        rollers.run();
        coro.yield();
      }
    }).named("Intaking"));
    var outtaking = machine.addState(Command.noRequirements(coro -> {
      coro.awaitAny(extend.extend()); // Extend first
      while (true) {
        rollers.reverse();
        coro.yield();
      }
    }).named("Intake Outtaking"));

    idling.switchTo(intaking).when(() -> Bindings.intake);
    idling.switchTo(outtaking).when(() -> Bindings.outtake);

    intaking.switchTo(idling).when(() -> !Bindings.intake && !Bindings.outtake);
    intaking.switchTo(outtaking).when(() -> Bindings.outtake);

    outtaking.switchTo(idling).when(() -> !Bindings.intake && !Bindings.outtake);
    outtaking.switchTo(intaking).when(() -> Bindings.intake);

    return machine;
  }
}
