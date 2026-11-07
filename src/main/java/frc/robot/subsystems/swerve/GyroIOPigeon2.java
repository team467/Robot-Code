package frc.robot.subsystems.swerve;

import static frc.robot.Schematic.pigeonCanId;
import static frc.robot.subsystems.swerve.DriveConstants.odometryFrequency;
import static org.wpilib.hardware.bus.CANPort.CAN_S0;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.Queue;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;

public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId, CANBus.systemcore(CAN_S0.value));
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<Angle> robotPitch = pigeon.getPitch();
  private final StatusSignal<Angle> robotRoll = pigeon.getRoll();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(odometryFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, yawVelocity, accelY, accelX, robotPitch, robotRoll);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = OdometryThread.getInstance().registerSignal(yaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY, robotRoll, robotPitch)
            .isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map(Rotation2d::fromDegrees)
            .toArray(Rotation2d[]::new);
    inputs.roll = robotRoll.getValueAsDouble();
    inputs.pitch = robotPitch.getValueAsDouble();

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
