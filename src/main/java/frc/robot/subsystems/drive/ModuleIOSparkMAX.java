package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotConstants;

public class ModuleIOSparkMAX implements ModuleIO {
  private final CANSparkMax driveMotor;
  private final RelativeEncoder driveEncoder;

  private final CANSparkMax turnMotor;
  private final RelativeEncoder turnEncoder;

  private final WPI_CANCoder turnEncoderAbsolute;

  private int resetCount = 0;
  private final int index;

  public ModuleIOSparkMAX(int driveMotorId, int turnMotorId, int turnAbsEncoderId, int index) {
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    turnEncoderAbsolute = new WPI_CANCoder(turnAbsEncoderId);

    // Convert rotations to radians
    double rotsToRads =
        Units.rotationsToRadians(1)
            * RobotConstants.get().moduleDriveGearRatio().getRotationsPerInput();
    driveEncoder.setPositionConversionFactor(rotsToRads);
    turnEncoder.setPositionConversionFactor(rotsToRads);

    // Convert rotations per minute to radians per second
    driveEncoder.setVelocityConversionFactor(rotsToRads / 60);
    turnEncoder.setVelocityConversionFactor(rotsToRads / 60);

    // Invert motors
    driveMotor.setInverted(false);
    turnMotor.setInverted(false);

    driveMotor.setIdleMode(IdleMode.kCoast);
    turnMotor.setIdleMode(IdleMode.kCoast);

    driveMotor.enableVoltageCompensation(12);
    turnMotor.enableVoltageCompensation(12);

    turnEncoderAbsolute.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();

    // Reset the turn encoder sometimes when not moving
    if (turnEncoder.getVelocity() < Units.degreesToRadians(0.5)) {
      if (++resetCount >= 500) {
        resetCount = 0;
        turnEncoder.setPosition(
            Rotation2d.fromDegrees(turnEncoderAbsolute.getAbsolutePosition())
                .minus(RobotConstants.get().absoluteAngleOffset()[index])
                .getRadians());
      }
    } else {
      resetCount = 0;
    }
    inputs.turnPositionRad = turnEncoder.getPosition();

    inputs.turnPositionAbsoluteRad =
        Rotation2d.fromDegrees(turnEncoderAbsolute.getAbsolutePosition())
            .minus(RobotConstants.get().absoluteAngleOffset()[index])
            .getRadians();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    turnMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
