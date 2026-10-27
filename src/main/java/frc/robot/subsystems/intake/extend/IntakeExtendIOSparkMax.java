package frc.robot.subsystems.intake.extend;

import static frc.robot.Schematic.intakeExtendCanId;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_LIMIT_ID;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POSITION_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VELOCITY_CONVERSION;
import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_EXTEND_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.intake.IntakeConstants.PID_D;
import static frc.robot.subsystems.intake.IntakeConstants.PID_I;
import static frc.robot.subsystems.intake.IntakeConstants.PID_P;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.command3.StateMachine;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;

public class IntakeExtendIOSparkMax implements IntakeExtendIO {

  private final SparkMax extendMotor;
  private final DigitalInput collapsedLimitSwitch;
  private final RelativeEncoder extendMotorEncoder;
  private final SparkClosedLoopController pidController;
  private boolean usingPID = false;

  public IntakeExtendIOSparkMax() {
    extendMotor = new SparkMax(CANPort.CAN_D0, intakeExtendCanId, MotorType.kBrushless);

    pidController = extendMotor.getClosedLoopController();
    extendMotorEncoder = extendMotor.getEncoder();

    extendMotor.configure(
        getSparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    collapsedLimitSwitch = new DigitalInput(EXTEND_LIMIT_ID);
  }

  public SparkMaxConfig getSparkMaxConfig() {
    EncoderConfig extendEnc = new EncoderConfig();
    var extendConfig = new SparkMaxConfig();
    extendConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit((int) Math.round(INTAKE_EXTEND_MOTOR_CURRENT_LIMIT))
        .closedLoop
        .pid(PID_P, PID_I, PID_D)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    extendConfig.encoder.apply(extendEnc);
    return extendConfig;
  }

  public StateMachine stateMachine() {

  }
}
