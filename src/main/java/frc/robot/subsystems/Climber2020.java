package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorType;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;

public class Climber2020 extends SubsystemBase {
    private MotorControllerEncoder climberMotor = MotorControllerFactory.create(RobotConstants.get().climber2020MotorId(), MotorType.SPARK_MAX_BRUSHLESS);
    private boolean enabled = false;

    public Climber2020() {
        super();

        climberMotor.setInverted(RobotConstants.get().climber2020MotorInverted());
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        if (this.isEnabled()) {
            climberMotor.set(RobotConstants.get().climber2020UpSpeed());
        }
    }

    public void down() {
        if (this.isEnabled()) {
            climberMotor.set(-RobotConstants.get().climber2020DownSpeed());
        }
    }

    public void stop() {
        climberMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Climber Position", () -> climberMotor.getPosition(), null);
        builder.addDoubleProperty("Climber Velocity", () -> climberMotor.getVelocity(), null);
    }
}

