package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  class ArmIOInputs {
    public double extendPosition = 0.0;
    public double extendVelocity = 0.0;
    public double extendAppliedVolts = 0.0;
    public double extendCurrent = 0.0;
    public double extendTemp = 0.0;

    /** Uses Lidar to get the absolute position of the arm. This is used to calibrate the arm. */
    public double extendPositionAbsolute = 0.0;

    public double rotatePositionAbsolute = 0.0;
    public double rotatePosition = 0.0;
    public double rotateVelocity = 0.0;
    public double rotateAppliedVolts = 0.0;
    public double rotateCurrent = 0.0;
    public double rotateTemp = 0.0;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setExtendVoltage(double volts) {}

  default void setRotateVoltage(double volts) {}

  default void setExtendBrakeMode(boolean brake) {}

  default void setRotateBrakeMode(boolean brake) {}

  public CANSparkMax getExtendMotor();

  public CANSparkMax getRotateMotor();
}
