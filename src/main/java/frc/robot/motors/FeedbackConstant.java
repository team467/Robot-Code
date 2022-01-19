package frc.robot.motors;

public class FeedbackConstant {
    private double kP;
    private double kI;
    private double kD;

    public FeedbackConstant(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }
}
