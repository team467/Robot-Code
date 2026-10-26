package frc.robot.subsystems.shooter;

import static frc.robot.Schematic.shooterBottomLeftMotorCanId;
import static frc.robot.Schematic.shooterBottomRightMotorCanId;
import static frc.robot.Schematic.shooterTopLeftMotorCanId;
import static frc.robot.Schematic.shooterTopRightMotorCanId;
import static frc.robot.subsystems.shooter.ShooterConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.shooter.ShooterConstants.IDLE_MODE;
import static frc.robot.subsystems.shooter.ShooterConstants.KA;
import static frc.robot.subsystems.shooter.ShooterConstants.KS;
import static frc.robot.subsystems.shooter.ShooterConstants.KV;
import static frc.robot.subsystems.shooter.ShooterConstants.VOLTAGE_COMPENSATION;
import static org.wpilib.hardware.bus.CANPort.CAN_D0;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Minute;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Revolutions;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;
import frc.lib.mechanisms.flywheels.FeedForwardFlywheel;
import frc.robot.Bindings;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

@AutoLog
public class ShooterSpark implements Shooter {

  private final SparkMax topLeftMotor;
  private final SparkMax topRightMotor;
  private final SparkMax bottomLeftMotor;
  private final SparkMax bottomRightMotor;

  private final Signal<Double> velocity;

  private final FeedForwardFlywheel flywheel;

  public ShooterSpark() {
    flywheel = new FeedForwardFlywheel(KS, KV, KA, this::setVoltage, this::getVelocity);

    topLeftMotor = new SparkMax(CAN_D0, shooterTopLeftMotorCanId, MotorType.kBrushless);
    topRightMotor = new SparkMax(CAN_D0, shooterTopRightMotorCanId, MotorType.kBrushless);
    bottomLeftMotor = new SparkMax(CAN_D0, shooterBottomLeftMotorCanId, MotorType.kBrushless);
    bottomRightMotor = new SparkMax(CAN_D0, shooterBottomRightMotorCanId, MotorType.kBrushless);

    var bottomLeftMotorConfig = new SparkMaxConfig();
    bottomLeftMotorConfig
        .inverted(false)
        .idleMode(IDLE_MODE)
        .voltageCompensation(VOLTAGE_COMPENSATION)
        .smartCurrentLimit(CURRENT_LIMIT)
        .secondaryCurrentLimit(CURRENT_LIMIT);
    bottomLeftMotor.configure(bottomLeftMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    topRightMotor.configure(bottomLeftMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    var bottomRightMotorConfig = new SparkMaxConfig().apply(bottomLeftMotorConfig).inverted(true);
    bottomRightMotor.configure(bottomRightMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    topLeftMotor.configure(bottomRightMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    var encoder = topLeftMotor.getEncoder();
    velocity = encoder.getVelocity();
  }

  @Override
  public boolean atSetpoint() {
    return flywheel.atSetpoint();
  }

  @Override
  public void stop() {
    flywheel.stop();
  }

  @Override
  public void shoot(AngularVelocity velocity) {
    flywheel.runAtVelocity(velocity);
  }

  @Override
  public void shoot(Distance distance) {
    flywheel.runAtVelocity(calculateSetpoint(distance));
  }

  public void setVoltage(Voltage voltage) {
    topLeftMotor.setVoltage(Volts.zero());
    topRightMotor.setVoltage(Volts.zero());
    bottomLeftMotor.setVoltage(Volts.zero());
    bottomRightMotor.setVoltage(Volts.zero());
  }

  public AngularVelocity getVelocity() {
    return Revolutions.per(Minute).of(velocity.get());
  }

  // TODO: empirically determine the relationship between distance and air time
  public Time getAirTimeSeconds(Distance distance) {
    return Seconds.of(0.0617 * distance.in(Meters) + 0.872);
  }

  /**
   * Calculates a setpoint (in radians per second) based on distance
   *
   * @param distance Distance
   * @return Setpoint in radians per second
   */
  private AngularVelocity calculateSetpoint(Distance distance) {
    // calculate rpm depending on distance
    AngularVelocity velocity =
        RPM.of(
            25.30184 * Math.pow(distance.in(Meters), 2)
                - 47.12642 * distance.in(Meters)
                + 1001.70713);
    if (velocity.gt(RPM.of(1550))) {
      return RPM.of(1550);
    } else {
      return velocity;
    }
  }

  public StateMachine stateMachine() {
    StateMachine machine = new StateMachine("Shooter");

    State idle = machine.addState(flywheel.run(coro -> {
          while (true) {
            flywheel.stop();
            coro.yield();
          }
        }
    ).named("Shooter Idle"));
    State speedingUp = machine.addState(flywheel.run(coro -> {
      while (!flywheel.atSetpoint()) {
        flywheel.runAtVelocity(Bindings.shooterSetpoint);
        coro.yield();
      }
    }).named("Shooter Speed Up"));
    State shooting = machine.addState(flywheel.run(coro -> {
      while (flywheel.atSetpoint()) {
        flywheel.runAtVelocity(Bindings.shooterSetpoint);
        coro.yield();
      }
    }).named("Shooter Shoot"));

    idle.switchTo(speedingUp).when(() -> Bindings.shoot);

    speedingUp.switchTo(idle).when(() -> !Bindings.shoot);
    speedingUp.switchTo(shooting).whenComplete();

    shooting.switchTo(idle).when(() -> !Bindings.shoot);
    shooting.switchTo(speedingUp).whenComplete();

    shooting.onEnter(() -> Bindings.shooterIsShooting = true);
    shooting.onExit(() -> Bindings.shooterIsShooting = false);

    return machine;
  }
}
