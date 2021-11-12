package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber2020 extends SubsystemBase {
    private CANSparkMax climberMotor = new CANSparkMax(Constants.CLIMBER2020_MOTOR_ID, MotorType.kBrushless);
    private boolean enabled = false;

    public Climber2020() {
        super();

        climberMotor.setInverted(Constants.CLIMBER2020_MOTOR_INVERTED);
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
        if (this.isEnabled())
            climberMotor.set(Constants.CLIMBER2020_UP_SPEED);
    }

    public void down() {
        if (this.isEnabled())
            climberMotor.set(-Constants.CLIMBER2020_DOWN_SPEED);
    }

    public void stop() {
        climberMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty(".climber_position", () -> climberMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty(".climber_speed", () -> climberMotor.getEncoder().getVelocity(), null);
    }
}

