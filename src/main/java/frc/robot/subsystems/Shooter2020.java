package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorType;
import frc.robot.motors.SpeedControllerEncoder;
import frc.robot.motors.SpeedControllerFactory;

public class Shooter2020 extends SubsystemBase {
    private SpeedControllerGroup flywheelMotorGroup;
    private SpeedControllerEncoder flywheelMotorLeader;
    private SpeedControllerEncoder flywheelMotorFollower;
    private SpeedControllerEncoder triggerMotor;

    public Shooter2020() {
        super();

        flywheelMotorLeader = SpeedControllerFactory.create(RobotConstants.get().shooter2020FlywheelLeaderMotorId(), MotorType.TALON_SRX);
        flywheelMotorLeader.setInverted(RobotConstants.get().shooter202FlywheelLeaderInverted());

        if (RobotConstants.get().shooter2020FlywheelDualMotors()) {
            flywheelMotorFollower = SpeedControllerFactory.create(RobotConstants.get().shooter2020FlywheelFollowerMotorId(), MotorType.TALON_SRX);
            flywheelMotorFollower.setInverted(RobotConstants.get().shooter202FlywheelFollowerInverted());
            flywheelMotorGroup = new SpeedControllerGroup(flywheelMotorLeader, flywheelMotorFollower);
        } else {
            flywheelMotorGroup = new SpeedControllerGroup(flywheelMotorLeader);
        }

        triggerMotor = SpeedControllerFactory.create(RobotConstants.get().shooter2020TriggerMotorId(), MotorType.TALON_SRX);
        triggerMotor.setInverted(RobotConstants.get().shooter2020TriggerInverted());
    }

    public void setFlywheel(double speed) {
        flywheelMotorGroup.set(speed); // TODO PIDS, all of them -_-
    }

    public void setFlywheelDefault() {
        setFlywheel(RobotConstants.get().shooter2020FlywheelDefaultSpeed());
    }

    public void stopFlywheel() {
        flywheelMotorGroup.set(0.0);
    }

    public void triggerForward() {
        triggerMotor.set(1.0);
    }

    public void triggerBackward() {
        triggerMotor.set(-1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty(".flywheel_speed", () -> flywheelMotorLeader.getVelocity(), null);
        builder.addDoubleProperty(".trigger_speed", () -> triggerMotor.getVelocity(), null);
    }
}


