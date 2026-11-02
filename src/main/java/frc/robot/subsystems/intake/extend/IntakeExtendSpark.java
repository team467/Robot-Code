package frc.robot.subsystems.intake.extend;

import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.HOME_VOLTAGE;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakeConstants;
import org.wpilib.command3.Command;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;

public class IntakeExtendSpark implements IntakeExtend {

  private final SparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final SparkClosedLoopController pidController;
  private final DigitalInput limitSwitch;

  public IntakeExtendSpark() {
    extendMotor = new SparkMax(CANPort.CAN_D0, intakeExtendCanId, MotorType.kBrushless);

    pidController = extendMotor.getClosedLoopController();

    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit((int) Math.round(INTAKE_EXTEND_MOTOR_CURRENT_LIMIT))
        .closedLoop
        .pid(PID_P, PID_I, PID_D)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    extendMotor.configure(
        extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendEncoder = extendMotor.getEncoder();

    limitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  private boolean notAtSetpoint() {
    return !Rotations.of(extendEncoder.getPosition().get())
        .equals(Rotations.of(IntakeConstants.EXTEND_POS));
  }

  @Override
  public Command extend() {
    return run(coro -> {
      while (notAtSetpoint()) {
        pidController.setSetpoint(IntakeConstants.EXTEND_POS, ControlType.kPosition);
        coro.yield();
      }
    }).named("Intake Extending");
  }

  @Override
  public Command retract() {
    return run(coro -> {
      while (true) {
        while (!limitSwitch.get()) {
          extendMotor.setVoltage(HOME_VOLTAGE);
          coro.yield();
        }
        while (limitSwitch.get()) {
          extendMotor.setVoltage(Volts.zero());
          coro.yield();
        }
      }
    }).named("Intake Retracting");
  }
}
