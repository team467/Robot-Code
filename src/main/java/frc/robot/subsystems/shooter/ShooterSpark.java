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
import static org.wpilib.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Bindings;
import java.util.Optional;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.epilogue.Logged;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

@Logged
public class ShooterSpark implements Shooter, Mechanism {

  private final SparkMax topLeftMotor;
  private final SparkMax topRightMotor;
  private final SparkMax bottomLeftMotor;
  private final SparkMax bottomRightMotor;
  private final SparkClosedLoopController controller;

  public ShooterSpark(Optional<ShooterSim> sim) {
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
    bottomLeftMotorConfig.closedLoop.feedForward.kS(KS).kA(KA).kV(KV);

    bottomLeftMotor.configure(bottomLeftMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    controller = bottomLeftMotor.getClosedLoopController();

    topRightMotor.configure(new SparkMaxConfig().follow(bottomLeftMotor),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    bottomRightMotor.configure(new SparkMaxConfig().follow(bottomLeftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    topLeftMotor.configure(new SparkMaxConfig().follow(bottomLeftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    if (sim.isPresent()) {
      sim.get().init(bottomLeftMotor);
      Scheduler.getDefault().addPeriodic(sim.get()::getPeriodic);
    }
  }

  /**
   * Calculates a setpoint (in radians per second) based on distance
   *
   * @param distance Distance
   * @return Setpoint in radians per second
   */
  public static AngularVelocity calculateSetpoint(Distance distance) {
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

    State idle = machine.addState(run(coro -> {
          while (true) {
            bottomLeftMotor.stopMotor();
            coro.yield();
          }
        }
    ).named("Shooter Idle"));
    State speedingUp = machine.addState(run(coro -> {
      while (!controller.isAtSetpoint().get(false)) {
        controller.setSetpoint(Bindings.shooterSetpoint.in(RPM), ControlType.kVelocity);
        coro.yield();
      }
    }).named("Shooter Speed Up"));
    State shooting = machine.addState(run(coro -> {
      while (controller.isAtSetpoint().get(false)) {
        controller.setSetpoint(Bindings.shooterSetpoint.in(RPM), ControlType.kVelocity);
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

    machine.setInitialState(idle);

    return machine;
  }
}
