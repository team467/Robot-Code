package frc.robot.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface MotorControllerEncoder extends MotorController {
    public double getPosition();
    public double getVelocity();
    public void resetPosition();
    public double getCurrent();
    public void setUnitsPerRotation(double unitsPerRotation);
}
