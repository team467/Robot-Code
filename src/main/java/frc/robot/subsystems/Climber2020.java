package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber2020 extends SubsystemBase {
    CANSparkMax spark1 = new CANSparkMax(Constants.CLIMBER2020_MOTOR_ID, MotorType.kBrushless);
    boolean enabled = false;

    public Climber2020() {
        super();
    }

    public void enable() {
        enabled = true;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        spark1.set(Constants.CLIMBER2020_SPEED);
    }

    public void down() {
        spark1.set(-Constants.CLIMBER2020_SPEED);
    }

    public void stop() {
        spark1.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty(".climber_position", () -> spark1.getEncoder().getPosition(), null);
        builder.addDoubleProperty(".climber_speed", () -> spark1.getEncoder().getVelocity(), null);
    }
}

