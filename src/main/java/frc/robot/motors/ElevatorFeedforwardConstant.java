package frc.robot.motors;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorFeedforwardConstant {
    private final double kS;
    private final double kG;
    private final double kV;
    private final double kA;

    public ElevatorFeedforwardConstant(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    public ElevatorFeedforwardConstant(double kS, double kG, double kV) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = 0;
    }

    public double getkS() {
        return kS;
    }

    public double getkG() {
        return kG;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public ElevatorFeedforward getFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }
}
