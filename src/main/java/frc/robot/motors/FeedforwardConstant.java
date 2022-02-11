package frc.robot.motors;

public class FeedforwardConstant {
    private double kS;
    private double kV;
    private double kA;

    public FeedforwardConstant(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double getkS() {
        return kS;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }
}
