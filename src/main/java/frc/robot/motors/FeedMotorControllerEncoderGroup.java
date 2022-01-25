package frc.robot.motors;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** Allows multiple {@link MotorController} objects to be linked together. */
public class FeedMotorControllerEncoderGroup implements MotorController, Sendable, AutoCloseable {
    private boolean m_isInverted;
    private final MotorControllerEncoder[] m_motorControllers;
    private static int instances;
    private SimpleMotorFeedforward forwardFF;
    private SimpleMotorFeedforward backwardFF;
    private PIDController forwardVelocityFB;
    private PIDController backwardVelocityFB;
    private PIDController forwardPositionFB;
    private PIDController backwardPositionFB;
    private double maxSpeed;
    private double maxAcceleration;

    /**
     * Create a new MotorControllerGroup with the provided MotorControllers and feed
     * constants.
     *
     * @param motorController  The first MotorControllerEncoder to add
     * @param motorControllers The MotorControllerEncoder to add
     */
    public FeedMotorControllerEncoderGroup(
            MotorControllerEncoder motorController, MotorControllerEncoder... motorControllers) {
        m_motorControllers = new MotorControllerEncoder[motorControllers.length + 1];
        m_motorControllers[0] = motorController;
        System.arraycopy(motorControllers, 0, m_motorControllers, 1, motorControllers.length);
        init();
    }

    public FeedMotorControllerEncoderGroup(MotorControllerEncoder[] motorControllers) {
        m_motorControllers = Arrays.copyOf(motorControllers, motorControllers.length);
        init();
    }

    public void initFF(FeedforwardConstant forward, FeedforwardConstant backward, double maxSpeed, double maxAcceleration) {
        this.forwardFF = new SimpleMotorFeedforward(forward.getkS(), forward.getkV(), forward.getkA());
        this.backwardFF = new SimpleMotorFeedforward(backward.getkS(), backward.getkV(), backward.getkA());
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
    }

    public void initFB(FeedbackConstant forwardVelocity, FeedbackConstant backwardVelocity,
            FeedbackConstant forwardPosition,
            FeedbackConstant backwardPosition) {
        this.forwardVelocityFB = new PIDController(forwardVelocity.getkP(), 0.0, forwardVelocity.getkD());
        this.backwardVelocityFB = new PIDController(backwardVelocity.getkP(), 0.0, backwardVelocity.getkD());
        this.forwardPositionFB = new PIDController(forwardPosition.getkP(), 0.0, forwardPosition.getkD());
        this.backwardPositionFB = new PIDController(backwardPosition.getkP(), 0.0, backwardPosition.getkD());
    }

    private void init() {
        for (MotorControllerEncoder controller : m_motorControllers) {
            SendableRegistry.addChild(this, controller);
        }
        instances++;
        SendableRegistry.addLW(this, "FeedMotorControllerGroup", instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    public void set(double speed, boolean velocity) {
        double setpoint = (m_isInverted ? -speed : speed) * maxSpeed;
        boolean forward = setpoint >= 0;
        double currentVelocity = m_motorControllers[0].getVelocity();
        double time = Math.abs(setpoint - currentVelocity)/maxAcceleration;
        double ffVoltage = 0;
        if (time >= 0.5) {
            ffVoltage = forward ? forwardFF.calculate(currentVelocity, setpoint, time) : backwardFF.calculate(currentVelocity, setpoint, time);
        } else {
            ffVoltage = forward ? forwardFF.calculate(setpoint) : backwardFF.calculate(setpoint);
        }
        PIDController selectedController = null;
        if (velocity) {
            if (forward) {
                selectedController = forwardVelocityFB;
            } else {
                selectedController = backwardVelocityFB;
            }
        } else {
            if (forward) {
                selectedController = forwardPositionFB;
            } else {
                selectedController = backwardPositionFB;
            }
        }

        double adjustedVoltage = ffVoltage;
        if (selectedController != null) {
            adjustedVoltage += selectedController.calculate(m_motorControllers[0].getVelocity(), setpoint);
        }
        for (MotorControllerEncoder motorController : m_motorControllers) {
            motorController.setVoltage(adjustedVoltage);
        }
    }

    @Override
    public void set(double speed) {
        if (forwardFF == null && forwardVelocityFB == null) {
            for (MotorControllerEncoder motorController : m_motorControllers) {
                motorController.set(speed);
            }
        } else {
            set(speed, true);
        }
    }

    @Override
    public double get() {
        if (m_motorControllers.length > 0) {
            return m_motorControllers[0].get() * (m_isInverted ? -1 : 1);
        }
        return 0.0;
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_isInverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return m_isInverted;
    }

    @Override
    public void disable() {
        for (MotorControllerEncoder motorController : m_motorControllers) {
            motorController.disable();
        }
    }

    @Override
    public void stopMotor() {
        for (MotorControllerEncoder motorController : m_motorControllers) {
            motorController.stopMotor();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}
