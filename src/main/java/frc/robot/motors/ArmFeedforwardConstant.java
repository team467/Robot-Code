package frc.robot.motors;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmFeedforwardConstant {
    private final double kS;
    private final double kCos;
    private final double kV;
    private final double kA;

    public ArmFeedforwardConstant(double kS, double kCos, double kV, double kA) {
        this.kS = kS;
        this.kCos = kCos;
        this.kV = kV;
        this.kA = kA;
    }

    public ArmFeedforwardConstant(double kS, double kCos, double kV) {
        this.kS = kS;
        this.kCos = kCos;
        this.kV = kV;
        this.kA = 0;
    }

    public double getkS() {
        return kS;
    }

    public double getkCos() {
        return kCos;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public ArmFeedforward getFeedforward() {
        return new ArmFeedforward(kS, kCos, kV, kA);
    }
}
