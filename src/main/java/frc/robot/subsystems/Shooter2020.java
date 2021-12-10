package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
    private PIDController flywheelPIDController;
    private SimpleMotorFeedforward flywheelFFController;
    private SpeedControllerEncoder triggerMotor;

    public Shooter2020() {
        super();

        flywheelMotorLeader = SpeedControllerFactory.create(RobotConstants.get().shooter2020FlywheelLeaderMotorId(), RobotConstants.get().shooter2020MotorType());
        flywheelMotorLeader.setInverted(RobotConstants.get().shooter202FlywheelLeaderInverted());

        if (RobotConstants.get().shooter2020FlywheelDualMotors()) {
            flywheelMotorFollower = SpeedControllerFactory.create(RobotConstants.get().shooter2020FlywheelFollowerMotorId(), RobotConstants.get().shooter2020MotorType());
            flywheelMotorFollower.setInverted(RobotConstants.get().shooter202FlywheelFollowerInverted());
            flywheelMotorGroup = new SpeedControllerGroup(flywheelMotorLeader, flywheelMotorFollower);
        } else {
            flywheelMotorGroup = new SpeedControllerGroup(flywheelMotorLeader);
        }

        flywheelPIDController = new PIDController(RobotConstants.get().shooter2020FlywheelkP(), RobotConstants.get().shooter2020FlywheelkI(), RobotConstants.get().shooter2020FlywheelkD(), RobotConstants.get().shooter2020FlywheelkF());
        flywheelFFController = new SimpleMotorFeedforward(-0.143, 0.13, 0.0062);

        triggerMotor = SpeedControllerFactory.create(RobotConstants.get().shooter2020TriggerMotorId(), MotorType.TALON_SRX);
        triggerMotor.setInverted(RobotConstants.get().shooter2020TriggerInverted());
    }

    public void setFlywheel(double speed) {
        if (RobotConstants.get().shooter2020FlywheelUseVelocity()) {
            double setpoint = speed * RobotConstants.get().shooter2020FlywheelkMaxVelocity();
            double output = flywheelPIDController.calculate(flywheelMotorLeader.getVelocity(), setpoint);
            double ff = flywheelFFController.calculate(setpoint/60);
            flywheelMotorGroup.setVoltage(output + ff);
        } else {
            setFlywheelRaw(speed);
        }
    }

    public void setFlywheelRaw(double speed) {
        flywheelMotorGroup.set(speed);
    }
 
    public void setFlywheelVelocity(double velocity) {
        flywheelMotorGroup.set(flywheelPIDController.calculate(flywheelMotorLeader.getVelocity(), velocity));
    }

    public void setFlywheelDefault() {
        setFlywheel(RobotConstants.get().shooter2020FlywheelDefaultSpeed());
    }

    public void stopFlywheel() {
        flywheelMotorGroup.set(0.0);
    }

    public void stopTrigger() {
        triggerMotor.set(0.0);
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

        builder.addDoubleProperty("Flywheel Velocity", () -> flywheelMotorLeader.getVelocity(), null);
        builder.addDoubleProperty("Flywheel Velo2", () -> flywheelMotorFollower.getVelocity(), null);
        builder.addDoubleProperty("Trigger Velocity", () -> triggerMotor.getVelocity(), null);
    }
}


