package frc.robot.motors;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedforwardConstant {
    private final double kS;
    private final double kV;
    private final double kA;

    public FeedforwardConstant(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public FeedforwardConstant(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
        this.kA = 0;
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

    public SimpleMotorFeedforward getSimpleMotorFeedforward() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }
}
