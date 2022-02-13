package frc.robot.motors;

import edu.wpi.first.math.controller.PIDController;

public class FeedbackConstant {
    private final double kP;
    private final double kD;

    public FeedbackConstant(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    public FeedbackConstant(double kP) {
        this.kP = kP;
        this.kD = 0;
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
}
