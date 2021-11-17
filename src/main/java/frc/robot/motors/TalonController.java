package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonController implements SpeedControllerEncoder {
    WPI_TalonSRX talon;
    
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
    public void pidWrite(double output) {
        talon.pidWrite(output);
    }

    @Override
    public void setP(double kP) {
        talon.config_kP(0, kP);       
    }

    @Override
    public void setI(double kI) {
        talon.config_kI(0, kI);       
    }

    @Override
    public void setD(double kD) {
        talon.config_kD(0, kD);       
        
    }

    @Override
    public void setF(double kF) {
        talon.config_kF(0, kF);       
    }

    @Override
    public void set(double value, ControlType controlType) {
        switch (controlType) {
        case Current:
            talon.set(ControlMode.Current, value);
            break;
        case PercentOutput:
            talon.set(ControlMode.PercentOutput, value);
            break;
        case Position:
            talon.set(ControlMode.Position, value);
            break;
        case Velocity:
            talon.set(ControlMode.Velocity, value);
            break;
        }        
    }

    @Override
    public double getPosition() {
        return talon.getSelectedSensorPosition();
    }

    @Override
    public double getVelocity() {
        return talon.getSelectedSensorVelocity();
    }

    @Override
    public double getCurrent() {
        return talon.getStatorCurrent();
    }
    
}
