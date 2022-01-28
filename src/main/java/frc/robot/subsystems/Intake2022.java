package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

public class Intake2022 extends SubsystemBase {

    private MotorControllerEncoder intakeMotor;


    public Intake2022() {
        super();
        intakeMotor = MotorControllerFactory.create(RobotConstants.get().intake2022MotorID(), MotorType.TALON_SRX);
    }


    public void in() {
        intakeMotor.set(RobotConstants.get().intake2022InSpeed());
    }

    public void out() {
        intakeMotor.set(-RobotConstants.get().intake2022OutSpeed());
    }

    public void stop() {
        intakeMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Intake Position", () -> intakeMotor.getPosition(), null);
        builder.addDoubleProperty("Intake Velocity", () -> intakeMotor.getVelocity(), null);
    }

}
