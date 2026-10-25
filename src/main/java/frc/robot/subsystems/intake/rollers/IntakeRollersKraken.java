package frc.robot.subsystems.intake.rollers;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_VOLTS;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.lib.mechanisms.flywheels.KrakenFlywheel;
import frc.robot.Schematic;
import org.wpilib.command3.Command;
import org.wpilib.hardware.bus.CANPort;

public class IntakeRollersKraken implements IntakeRollers {

  KrakenFlywheel flywheel;

  public IntakeRollersKraken() {
    flywheel = new KrakenFlywheel("Intake Rollers", CANPort.CAN_D0, Schematic.intakeMotorCanId,
        InvertedValue.CounterClockwise_Positive, INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT, INTAKE_VOLTS);
  }

  @Override
  public Command feed() {
    return flywheel.run();
  }

  @Override
  public Command reverse() {
    return flywheel.reverse();
  }

  @Override
  public Command stop() {
    return flywheel.stop();
  }
}
