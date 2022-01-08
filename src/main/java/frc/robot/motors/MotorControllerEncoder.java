package frc.robot.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface MotorControllerEncoder extends MotorController {
    public double getPosition();
    public double getVelocity();
    public double getCurrent();
}
