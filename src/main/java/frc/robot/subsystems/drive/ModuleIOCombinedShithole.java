package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ModuleIOCombinedShithole implements ModuleIO {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;
  private final CANcoder turnEncoderAbsolute;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isDriveMotorInverted = true;
  private final boolean isTurnMotorInverted = false;

  private final int index;

  private double rotsToMeters;
  private double rotsToRads;

  public ModuleIOCombinedShithole(int index) {
    int driveMotorId;
    int turnMotorId;
    int turnAbsEncoderId;
    switch (index) {
      case 0 -> {
        driveMotorId = Schematic.FRONT_LEFT_DRIVE_ID;
        turnMotorId = Schematic.FRONT_LEFT_STEERING_ID;
        turnAbsEncoderId = Schematic.FRONT_LEFT_CANCODER_ID;
      }
      case 1 -> {
        driveMotorId = Schematic.FRONT_RIGHT_DRIVE_ID;
        turnMotorId = Schematic.FRONT_RIGHT_STEERING_ID;
        turnAbsEncoderId = Schematic.FRONT_RIGHT_CANCODER_ID;
      }
      case 2 -> {
        driveMotorId = Schematic.REAR_LEFT_DRIVE_ID;
        turnMotorId = Schematic.REAR_LEFT_STEERING_ID;
        turnAbsEncoderId = Schematic.REAR_LEFT_CANCODER_ID;
      }
      case 3 -> {
        driveMotorId = Schematic.REAR_RIGHT_DRIVE_ID;
        turnMotorId = Schematic.REAR_RIGHT_STEERING_ID;
        turnAbsEncoderId = Schematic.REAR_RIGHT_CANCODER_ID;
      }
      default -> throw new IllegalArgumentException("Drive: Illegal index attempted " + index);
    }
    driveMotor = new TalonFX(driveMotorId);
    turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
    turnEncoderAbsolute = new CANcoder(turnAbsEncoderId);

    rotsToMeters =
        Units.rotationsToRadians(1)
            * (DriveConstants.WHEEL_DIAMETER / 2)
            * DriveConstants.DRIVE_GEAR_RATIO.getRotationsPerInput();
    rotsToRads =
        Units.rotationsToRadians(1) * DriveConstants.TURN_GEAR_RATIO.getRotationsPerInput();

    // TODO: supply or stator current limits?

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 50;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotor.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnMotor.setSmartCurrentLimit(40);
    turnMotor.enableVoltageCompensation(12.0);
    setTurnBrakeMode(true);

    turnEncoderAbsolute.getConfigurator().apply(new CANcoderConfiguration());

    turnAbsolutePosition = turnEncoderAbsolute.getPosition();
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent
    );
    driveMotor.optimizeBusUtilization();

    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition);

    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble() * rotsToMeters;
    inputs.drivePositionMeters = drivePosition.getValueAsDouble() * rotsToMeters;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnVelocityRadPerSec = turnMotor.getEncoder().getVelocity() * rotsToRads;
    inputs.turnPosition = Rotation2d.fromRotations(turnMotor.getEncoder().getPosition());
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(DriveConstants.ABSOLUTE_ANGLE_OFFSET[index]);
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    var config = new MotorOutputConfigs();
    config.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    turnMotor.setInverted(isDriveMotorInverted);
    turnMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
