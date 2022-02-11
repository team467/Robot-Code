package frc.robot.motors;

public class FeedbackConstant {
    private double kP;
    private double kD;

    public FeedbackConstant(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }
    
    public double getkP() {
        return kP;
    }

    public double getkD() {
        return kD;
    }
}
