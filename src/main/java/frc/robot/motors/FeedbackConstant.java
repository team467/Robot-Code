package frc.robot.motors;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class FeedbackConstant {
    private final double kP;
    private final double kD;

    public FeedbackConstant(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    public FeedbackConstant(double kP) {
        this(kP, 0);
    }
    
    public double getkP() {
        return kP;
    }

    public double getkD() {
        return kD;
    }

    public PIDController getPIDController() {
        return new PIDController(kP, 0, kD);
    }

    public ProfiledPIDController getProfiledPIDController(TrapezoidProfile.Constraints constraint) {
        return new ProfiledPIDController(kP, 0, kD, constraint);
    }
}
