package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

public class LlamaNeck2022 extends SubsystemBase {
    private static final Logger LOGGER = RobotLogManager.getMainLogger(LlamaNeck2022.class.getName());

    private MotorControllerEncoder llamaNeckMotor;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    public LlamaNeck2022() {
        super();

        llamaNeckMotor = MotorControllerFactory.create(RobotConstants.get().llamaNeck2022MotorID(), MotorType.TALON_SRX);
        upperLimitSwitch = new DigitalInput(RobotConstants.get().llamaNeck2022UpperLimitSwitchChannel());
        lowerLimitSwitch = new DigitalInput(RobotConstants.get().llamaNeck2022LowerLimitSwitchChannel());
    }

    public boolean getUpperLimitSwitch() {
        return upperLimitSwitch.get();
    }

    public boolean getLowerLimitSwitch() {
        return lowerLimitSwitch.get();
    }

    public void forward() {
        LOGGER.info("Starting llamaNeck, setting speed to " + RobotConstants.get().llamaNeck2022InSpeed());
        llamaNeckMotor.set(RobotConstants.get().llamaNeck2022InSpeed());
    }

    public void backward() {
        LOGGER.info("Reversing index, setting speed to " + RobotConstants.get().llamaNeck2022OutSpeed());
        llamaNeckMotor.set(-RobotConstants.get().llamaNeck2022OutSpeed());
    }

    public void stop() {
        LOGGER.info("Stopping llamaNeck, setting speed to 0");
        llamaNeckMotor.set(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Upper Limit Switch", () -> getUpperLimitSwitch(), null);
        builder.addBooleanProperty("Lower Limit Switch", () -> getLowerLimitSwitch(), null);
    }

}
