package frc.robot.motors;

import edu.wpi.first.wpilibj.SpeedController;

public interface SpeedControllerEncoder extends SpeedController {
    public double getPosition();
    public double getVelocity();
    public double getCurrent();
}
