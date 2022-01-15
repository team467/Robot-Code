package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorType;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;

public class Shooter2022 extends SubsystemBase {
    private MotorControllerGroup flywheelMotorGroup;
    private MotorControllerEncoder flywheelMotorLeader;
    private MotorControllerEncoder flywheelMotorFollower;
    private PIDController flywheelPIDController;
    private SimpleMotorFeedforward flywheelFFController;
    private MotorControllerEncoder triggerMotor;

    public Shooter2022() {
        super();

        flywheelMotorLeader = MotorControllerFactory.create(RobotConstants.get().shooter2020FlywheelLeaderMotorId(), RobotConstants.get().shooter2020MotorType());
        flywheelMotorLeader.setInverted(RobotConstants.get().shooter202FlywheelLeaderInverted());

        if (RobotConstants.get().shooter2020FlywheelDualMotors()) {
            flywheelMotorFollower = MotorControllerFactory.create(RobotConstants.get().shooter2020FlywheelFollowerMotorId(), RobotConstants.get().shooter2020MotorType());
            flywheelMotorFollower.setInverted(RobotConstants.get().shooter202FlywheelFollowerInverted());
            flywheelMotorGroup = new MotorControllerGroup(flywheelMotorLeader, flywheelMotorFollower);
        } else {
            flywheelMotorGroup = new MotorControllerGroup(flywheelMotorLeader);
        }

        flywheelPIDController = new PIDController(RobotConstants.get().shooter2020FlywheelkP(), RobotConstants.get().shooter2020FlywheelkI(), RobotConstants.get().shooter2020FlywheelkD());
        flywheelFFController = new SimpleMotorFeedforward(RobotConstants.get().shooter2020FlywheelkS(), RobotConstants.get().shooter2020FlywheelkV(), RobotConstants.get().shooter2020FlywheelkA());

        triggerMotor = MotorControllerFactory.create(RobotConstants.get().shooter2020TriggerMotorId(), MotorType.TALON_SRX);
        triggerMotor.setInverted(RobotConstants.get().shooter2020TriggerInverted());
    }

    public void setFlywheel(double speed) {
        if (RobotConstants.get().shooter2020FlywheelUseVelocity()) {
            double setpoint = speed * RobotConstants.get().shooter2020FlywheelkMaxVelocity();
            double output = flywheelPIDController.calculate(flywheelMotorLeader.getVelocity(), setpoint);
            double ff = flywheelFFController.calculate(setpoint);
            flywheelMotorGroup.setVoltage(output + ff);
        } else {
            flywheelMotorGroup.set(speed);
        }
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
        builder.addDoubleProperty("Flywheel Velocity Error", () -> flywheelPIDController.getSetpoint() - flywheelMotorLeader.getVelocity(), null);
        builder.addDoubleProperty("Trigger Velocity", () -> triggerMotor.getVelocity(), null);
    }
}
