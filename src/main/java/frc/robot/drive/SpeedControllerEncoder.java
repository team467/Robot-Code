package frc.robot.drive;

import edu.wpi.first.wpilibj.SpeedController;

public interface SpeedControllerEncoder extends SpeedController {
    public double getPosition();
    public double getVelocity();
}
