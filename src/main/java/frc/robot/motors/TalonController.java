package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonController implements MotorControllerEncoder {
    WPI_TalonSRX talon;
    private double unitsPerRotation = 1;
    
    public TalonController(int id) {
        talon = new WPI_TalonSRX(id);
    }

    @Override
    public void set(double speed) {
        talon.set(speed);
    }

    @Override
    public double get() {
        return talon.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        talon.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return talon.getInverted();
    }

    @Override
    public void disable() {
        talon.disable();
    }

    @Override
    public void stopMotor() {
        talon.stopMotor();
    }

    @Override
    public double getPosition() {
        return talon.getSelectedSensorPosition() * unitsPerRotation;
    }

    @Override
    public double getVelocity() {
        // The TalonSRX returns velocity in RPM. We want to work in revs per second instead.
        return (talon.getSelectedSensorVelocity()/60) * unitsPerRotation;
    }

    @Override
    public void resetPosition() {
        talon.setSelectedSensorPosition(0);
    }

    @Override
    public double getCurrent() {
        return talon.getStatorCurrent();
    }
    
    @Override
    public void setUnitsPerRotation(double unitsPerRotation) {
        this.unitsPerRotation = unitsPerRotation;
    }
}
