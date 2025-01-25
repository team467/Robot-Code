package frc.robot.subsystems.algae;

import static frc.lib.utils.SparkUtil.*;
import static frc.robot.Schematic.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {

  // motors
  private final SparkMax pivotMotor;
  private final SparkMax removalMotor;
  private final RelativeEncoder pivotMotorEncoder;
  private final RelativeEncoder removalMotorEncoder;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(algaePivotCanId, MotorType.kBrushless);
    removalMotor = new SparkMax(algaeRemovalCanId, MotorType.kBrushless);

    pivotMotorEncoder = pivotMotor.getEncoder();
    removalMotorEncoder = removalMotor.getEncoder();

    forwardLimitSwitch = pivotMotor.getForwardLimitSwitch();
    reverseLimitSwitch = pivotMotor.getReverseLimitSwitch();

    var pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(IdleMode.kBrake);

    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void setRemovalVolts(double volts) {
    System.out.print("setting removal motor to " + volts);
    removalMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    System.out.print("setting piv motor to " + volts);
    pivotMotor.setVoltage(volts);
  }

  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.removalVolts = removalMotor.getBusVoltage() * removalMotor.getAppliedOutput();
    inputs.pivotVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotVelocity = pivotMotorEncoder.getVelocity();
    inputs.removalVelocity = removalMotorEncoder.getVelocity();
    inputs.removalAmps = removalMotor.getOutputCurrent();
    inputs.pivotAmps = pivotMotor.getOutputCurrent();
    inputs.pivotPosition = pivotMotorEncoder.getPosition();

    inputs.isFullyExtended = forwardLimitSwitch.isPressed();
    System.out.print("Extend Limit switch works and is pressed" + forwardLimitSwitch.isPressed());

    inputs.isStowed = reverseLimitSwitch.isPressed();
    System.out.print("Stow Limit switch works and is pressed" + reverseLimitSwitch.isPressed());
  }
}
