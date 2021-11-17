package frc.robot.motors;

import edu.wpi.first.wpilibj.SpeedController;

public interface SpeedControllerEncoder extends SpeedController {
    public void setP(double kP);
    public void setI(double kI);
    public void setD(double kD);
    public void setF(double kF);

    public void set(double value, ControlType controlType);

    public double getPosition();
    public double getVelocity();
    public double getCurrent();
}
