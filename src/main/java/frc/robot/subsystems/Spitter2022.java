package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

public class Spitter2022 extends SubsystemBase {

  private final double THRESHOLD = 10;
  private MotorControllerEncoder spitterMotor;
  private PIDController spitterPIDController;
  private SimpleMotorFeedforward spitterFFController;

  public Spitter2022() {
    super();

    spitterMotor = MotorControllerFactory.create(RobotConstants.get().spitter2022MotorId(),
        MotorType.SPARK_MAX_BRUSHLESS);
    spitterMotor.setInverted(RobotConstants.get().spitter2022MotorInverted());

    spitterPIDController = new PIDController(
        RobotConstants.get().spitter2022FB().getkP(), 0,
        RobotConstants.get().spitter2022FB().getkD());
    spitterFFController = new SimpleMotorFeedforward(
        RobotConstants.get().spitter2022FF().getkS(), RobotConstants.get().spitter2022FF().getkV(),
        RobotConstants.get().spitter2022FF().getkA());
  }

  public void setSpeed(double speed) {
    if (RobotConstants.get().spitter2022UseVelocity()) {
      setVelocity(speed * RobotConstants.get().spitter2022MaxVelocity());
    } else {
      spitterMotor.set(speed);
    }
  }

  public void setVelocity(double velocity) {
    double output = spitterFFController.calculate(velocity);
    spitterPIDController.setSetpoint(velocity);
    if (RobotConstants.get().spitter2022UsePID()) {
      output += spitterPIDController.calculate(spitterMotor.getVelocity());
    }
    spitterMotor.setVoltage(output);
  }

  public void forward() {
    spitterMotor.set(RobotConstants.get().spitter2022ForwardSpeed());
  }

  public void backward() {
    spitterMotor.set(-RobotConstants.get().spitter2022BackwardSpeed());
  }

  public void stop() {
    spitterMotor.set(0.0);
  }

  public boolean atSpeed() {
    if (!RobotConstants.get().spitter2022UseVelocity()) {
      return true;
    }
    return Math.abs(spitterMotor.getVelocity() - spitterPIDController.getSetpoint()) <= THRESHOLD;

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Flywheel Velocity", () -> spitterMotor.getVelocity(), null);
    builder.addDoubleProperty("Flywheel Velocity Error",
        () -> spitterPIDController.getSetpoint() - spitterMotor.getVelocity(), null);
  }
}
