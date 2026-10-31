package frc.robot.subsystems.intake.extend;

import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.subsystems.intake.IntakeConstants.HOME_VOLTAGE;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Bindings;
import org.wpilib.command3.StateMachine;
import org.wpilib.hardware.bus.CANPort;

public class IntakeExtendSpark implements IntakeExtend {

  private final SparkMax extendMotor;
  private final SparkClosedLoopController pidController;

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
  }

  @Override
  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Intake Extend");

    var extending = machine.addState(
        run(coro -> {
          while (true) {
            pidController.setSetpoint(Bindings.intakePos.in(Rotations), ControlType.kPosition);
            coro.yield();
          }
        }).named("Intake Extend")
    );
    var resetting = machine.addState(
        run(coro -> {
          while (true) {
            extendMotor.setVoltage(HOME_VOLTAGE);
            coro.yield();
          }
        }).named("Intake Reset")
    );

    resetting.switchTo(extending).when(() -> Bindings.intakeExtend);
    extending.switchTo(resetting).when(() -> !Bindings.intakeExtend);

    machine.setInitialState(resetting); // Get pose at start of match

    return machine;
  }
}
