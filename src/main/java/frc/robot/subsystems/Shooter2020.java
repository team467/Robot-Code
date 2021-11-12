package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drive.MotorType;
import frc.robot.drive.SpeedControllerEncoder;
import frc.robot.drive.SpeedControllerFactory;

public class Shooter2020 extends SubsystemBase {
    private SpeedControllerEncoder flywheelMotor = SpeedControllerFactory.create(MotorType.TALON_SRX, RobotConstants.get().shooter2020FlywheelMotorId());

    public Shooter2020() {
        super();

        flywheelMotor.setInverted(RobotConstants.get().shooter202FlywheelInverted());
    }

    public void shootBall() {
        flywheelMotor.set(RobotConstants.get().shooter2020Speed());
    }

    public void reverseFlywheel() {
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


