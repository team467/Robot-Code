package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Climber2020 extends SubsystemBase {
    private CANSparkMax climberMotor = new CANSparkMax(RobotConstants.get().climber2020MotorId(), MotorType.kBrushless);
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
        if (this.isEnabled())
            climberMotor.set(RobotConstants.get().climber2020UpSpeed());
    }

    public void down() {
        if (this.isEnabled())
            climberMotor.set(-RobotConstants.get().climber2020DownSpeed());
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

